# Group 3 - Part 3: Autonomous Navigation

- World: group3_custom.world
- Map: group3_map.yaml
- Nav2 parameters: rubot_sw_ign.yaml (no changes)
- Initial pose: x=0.0, y=0.0, yaw=0.0

## Goals tested
1. Top-right area: reached
2. Bottom-left area: reached
3. Top-left area: reached

## Observations
- 2D Pose Estimate was used to correct the localization after restarting the simulation.
- The robot reached all three goals without touching the walls.
- The global path stayed inside the mapped free space and the localization remained consistent with the simulated pose in Gazebo.
- Issue found: group3_custom.world was not installed after git pull, so Gazebo could not find it. It was solved by rebuilding the package with `colcon build --symlink-install --packages-select my_robot_bringup`.
