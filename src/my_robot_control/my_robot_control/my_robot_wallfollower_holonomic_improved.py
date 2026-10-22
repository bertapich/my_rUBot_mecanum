#!/usr/bin/env python3
"""Improved holonomic right-wall follower with odometric obstacle bypass."""

import math
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan


class ImprovedHolonomicWallFollower(Node):
    """Follow right walls and distinguish short obstacles from long walls."""

    FOLLOW_RIGHT = 'FOLLOW_RIGHT'
    FOLLOW_FRONT = 'FOLLOW_FRONT'
    BYPASS_OBSTACLE = 'BYPASS_OBSTACLE'
    TURN_TO_RIGHT = 'TURN_TO_RIGHT'

    BYPASS_FRONT_RIGHT = 'BYPASS_FRONT_RIGHT'
    BYPASS_RIGHT = 'BYPASS_RIGHT'
    BYPASS_BACK_RIGHT = 'BYPASS_BACK_RIGHT'

    def __init__(self):
        super().__init__('improved_holonomic_wall_follower')

        # Controller and geometry parameters.
        self.follow_distance = float(
            self.declare_parameter('follow_distance', 0.35).value
        )
        self.max_front_length = float(
            self.declare_parameter('max_front_length', 0.50).value
        )
        self.forward_speed = float(
            self.declare_parameter('forward_speed', 0.30).value
        )
        self.lateral_speed = float(
            self.declare_parameter('lateral_speed', 0.40).value
        )
        self.distance_gain = float(
            self.declare_parameter('distance_gain', 0.80).value
        )
        self.angle_gain = float(
            self.declare_parameter('angle_gain', 1.20).value
        )
        self.max_angular_speed = float(
            self.declare_parameter('max_angular_speed', 1.00).value
        )
        self.detection_limit = float(
            self.declare_parameter('detection_limit', 1.20).value
        )
        self.back_right_limit_factor = float(
            self.declare_parameter('back_right_limit_factor', 1.50).value
        )

        self.state = self.FOLLOW_RIGHT
        self.bypass_phase = self.BYPASS_FRONT_RIGHT
        self.command = Twist()
        self.odom_pose = None
        self.front_start_pose = None
        self.last_scan = None
        self.last_odom = None
        self.stopped = False

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.publish_command)

        self.get_logger().info(
            'Improved holonomic wall follower ready.'
        )

    def publish_command(self):
        # Do not repeat an old movement if the sensors stop publishing.
        now = time.monotonic()
        if (self.stopped or self.last_scan is None
                or now - self.last_scan > 0.5
                or (self.state == self.FOLLOW_FRONT and
                    (self.last_odom is None or now - self.last_odom > 0.5))):
            self.command = Twist()
        self.publisher.publish(self.command)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = (position.x, position.y, q.x, q.y, q.z, q.w)
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if not all(math.isfinite(v) for v in values) or norm < 1e-6:
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        self.odom_pose = (position.x, position.y, yaw)
        self.last_odom = time.monotonic()

    def change_state(self, state):
        if state != self.state:
            self.get_logger().info(f'{self.state} -> {state}')
            self.state = state

    def scan_callback(self, scan):
        self.command = Twist()
        if self.stopped:
            return
        self.last_scan = time.monotonic()
        bounds = {'front': (-20, 20), 'front_right': (-70, -20),
                  'right': (-110, -70), 'back_right': (-160, -110)}
        rays = {name: [] for name in bounds}
        observed = {name: False for name in bounds}
        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            degrees = math.degrees(angle)
            for name, (low, high) in bounds.items():
                if low <= degrees < high or (name == 'front' and degrees == high):
                    # +inf denotes no return; it is never used as a distance.
                    if distance == math.inf:
                        observed[name] = True
                    elif (math.isfinite(distance) and
                          scan.range_min <= distance <= scan.range_max):
                        observed[name] = True
                        if distance <= self.detection_limit:
                            rays[name].append((distance, angle))
                    break

        # Average up to five nearest returns in each sector.
        regions = {}
        for name, values in rays.items():
            nearest = sorted(values)[:5]
            if nearest:
                regions[name] = (sum(v[0] for v in nearest)/len(nearest),
                                 sum(v[1] for v in nearest)/len(nearest))
        if not all(observed.values()) or not regions:
            return
        closest = min(regions, key=lambda name: regions[name][0])
        front = regions.get('front')
        blocked = front is not None and front[0] < 1.5*self.follow_distance
        odom_ready = (self.odom_pose is not None and self.last_odom is not None
                      and time.monotonic() - self.last_odom <= 0.5)

        if self.state == self.FOLLOW_RIGHT and blocked:
            if not odom_ready:
                return
            self.front_start_pose = self.odom_pose
            self.change_state(self.FOLLOW_FRONT)
        elif self.state == self.FOLLOW_FRONT:
            if not odom_ready or self.front_start_pose is None:
                return
            x0, y0, yaw0 = self.front_start_pose
            x, y, _ = self.odom_pose
            progress = abs(-math.sin(yaw0)*(x-x0) + math.cos(yaw0)*(y-y0))
            if not blocked:
                self.bypass_phase = self.BYPASS_FRONT_RIGHT
                self.change_state(self.BYPASS_OBSTACLE)
            elif progress >= self.max_front_length:
                self.change_state(self.TURN_TO_RIGHT)
        elif self.state == self.TURN_TO_RIGHT:
            if closest == 'right' and not blocked:
                self.change_state(self.FOLLOW_RIGHT)
        elif self.state == self.BYPASS_OBSTACLE:
            # Exactly one transition per scan, in the required order.
            if self.bypass_phase == self.BYPASS_FRONT_RIGHT and closest == 'right':
                self.bypass_phase = self.BYPASS_RIGHT
                self.get_logger().info(self.bypass_phase)
            elif self.bypass_phase == self.BYPASS_RIGHT and closest == 'back_right':
                self.bypass_phase = self.BYPASS_BACK_RIGHT
                self.get_logger().info(self.bypass_phase)
            elif self.bypass_phase == self.BYPASS_BACK_RIGHT and closest == 'right':
                self.change_state(self.FOLLOW_RIGHT)

        vx = vy = angular = 0.0
        if self.state == self.FOLLOW_RIGHT:
            if 'right' not in regions or blocked:
                return
            distance, angle = regions['right']
            vx = self.forward_speed
            vy = self.distance_gain*(self.follow_distance-distance)
            angular = self.angle_gain*(angle + math.pi/2)
        elif self.state == self.FOLLOW_FRONT:
            if front is None:
                return
            distance, angle = front
            vx = self.distance_gain*(distance-self.follow_distance)
            vy = self.lateral_speed
            angular = self.angle_gain*angle
        elif self.state == self.TURN_TO_RIGHT:
            angular = self.max_angular_speed
        elif self.state == self.BYPASS_OBSTACLE:
            if blocked:
                return  # Unexpected new frontal obstacle: stop.
            vx = 0.5*self.forward_speed
            if self.bypass_phase == self.BYPASS_FRONT_RIGHT:
                if closest not in ('front_right', 'right'):
                    return
                vy = 0.25*self.lateral_speed
            elif self.bypass_phase == self.BYPASS_RIGHT:
                if 'right' not in regions:
                    return
                vy = self.distance_gain*(self.follow_distance-regions['right'][0])
            elif self.bypass_phase == self.BYPASS_BACK_RIGHT:
                if 'back_right' not in regions:
                    return
                extra = max(0.0, regions['back_right'][0]
                            - self.back_right_limit_factor*self.follow_distance)
                vy = -(0.25*self.lateral_speed + self.distance_gain*extra)
            else:
                return
            # No angular correction: the closest region must move backwards.
        else:
            return
        self.command.linear.x = max(-self.forward_speed, min(self.forward_speed, vx))
        self.command.linear.y = max(-self.lateral_speed, min(self.lateral_speed, vy))
        self.command.angular.z = max(-self.max_angular_speed,
                                     min(self.max_angular_speed, angular))

    def stop(self):
        """Publish one zero command before destroying the node."""
        self.stopped = True
        self.command = Twist()
        self.timer.cancel()
        try:
            self.publisher.publish(self.command)
        except Exception:
            pass


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = ImprovedHolonomicWallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
