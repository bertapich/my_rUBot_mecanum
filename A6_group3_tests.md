# A6: Traffic-Sign Navigation: Part 4 (Group 3)

## Objective

Run and test the `traffic_sign_navigation` node in simulation (The Construct). The robot navigates to the inspection pose, waits for a message on `/traffic_sign` and chooses a route according to the sign received.

## Files used

| Type | File |
|---|---|
| World | `group3_custom.world` |
| Map | `group3_map.yaml` |
| Launch | `traffic_sign_navigation.launch.py` |
| Navigation code | `traffic_sign_navigation.py` |
| Group poses | `traffic_sign_navigation_sw_group3.yaml` |

## Poses (map frame)

| Pose | [x, y, yaw] |
|---|---|
| initial_pose | [0.0, 0.0, 0.0] |
| inspection_pose | [0.3, 0.0, 0.0] |
| left_waypoint | [1.45, 0.55, 0.0] |
| right_waypoint | [1.45, -1.85, 0.0] |
| final_pose | [1.8, -0.6, 0.0] |
| stop_duration | 3.0 s |

## How to run the test

1. Build the package:
   ```bash
   cd ~/my_rUBot_mecanum
   colcon build --packages-select my_robot_ai_navigation
   source install/setup.bash
   ```
2. Terminal 1, Gazebo:
   ```bash
   ros2 launch my_robot_bringup my_robot_bringup_gz.launch.py \
     world:=group3_custom.world \
     robot_model:=rubot/rubot_mecanum.urdf.xacro \
     x:=0.0 y:=0.0 yaw:=0.0
   ```
3. Terminal 2, Nav2:
   ```bash
   ros2 launch my_robot_navigation2 navigation2_robot.launch.py \
     use_sim_time:=true \
     map_file:=group3_map.yaml \
     params_file:=rubot_sw_lidar_ign_ai.yaml
   ```
4. Terminal 3, navigation node:
   ```bash
   ros2 launch my_robot_ai_navigation traffic_sign_navigation.launch.py \
     config_file:=traffic_sign_navigation_sw_group3.yaml
   ```
5. Terminal 4, simulate the YOLO detection:
   ```bash
   ros2 topic pub --once /traffic_sign std_msgs/msg/String "{data: left}"
   ```
   Repeat with `right` and `stop`, restarting the simulation and the node before each test.

## Expected behaviour

| Sign | Behaviour |
|---|---|
| `left` | Goes to `left_waypoint`, then to `final_pose` |
| `right` | Goes to `right_waypoint`, then to `final_pose` |
| `stop` | Waits `stop_duration`, then goes to `final_pose` |
| other | Stays at the inspection pose |

## Problems found and solutions

- **`ModuleNotFoundError: traffic_sign_navigation`**: `setup.py` expects `traffic_sign_navigation.py`, but the group code was saved as `traffic_sign_navigation_group3.py`. Solved by copying it with the correct name and rebuilding.
- **`--editable` error in `colcon build`**: incompatible `setuptools` version. Solved with `pip3 install setuptools==58.2.0`.
- **NumPy error in `tf_transformations`**: `transforms3d` is not compatible with NumPy 1.24 or later. Solved with `pip3 install "numpy==1.23.5"`.
- **AMCL localization lost and robot jumping in Gazebo**: an old `ign gazebo server` process from a previous session was still running, so two simulations were publishing `/tf`, `/odom` and `/scan` at the same time. Solved by killing old processes (`pkill -9 -f "ign gazebo"`) and checking with `ps aux` before each test.

## Results

- `left`: route through the left waypoint, reaches `final_pose`.
- `right`: route through the right waypoint, reaches `final_pose`.
- `stop`: waits 3 s at the inspection pose, then reaches `final_pose`.

The video with the three tests is submitted in the Campus Virtual.
