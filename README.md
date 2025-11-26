# JetCobot Workspace

## Requirements
- OS: Ubuntu 22.04 LTS
- ROS 2: Humble Hawksbill

This workspace targets Ubuntu 22.04 and ROS 2 Humble. Use these versions when building or running the project.

- Python 3 (system default)
- Dependencies (examples — install as needed):
    - pymycobot
        ```sh
        python3 -m pip install pymycobot
        ```

## Packages
This workspace contains three main packages:

- [play_wth_jecobot](src/play_wth_jecobot) (Python helpers / GUI)
- [hardware_jetcobot_pkg](src/hardware_jetcobot_pkg) (hardware bridge & ros2_control plugin and nodes)
- [moveit_jetcobot_pkg](src/moveit_jetcobot_pkg) (MoveIt configuration and planners)
- [jetcobot_urdf](src/jetcobot_urdf) (Robot urdf and rviz visualization display)

## Notes
The C++ planner helper (can plan the path according to the target pose or angles) which can be found on [.../src/move_to_target_pose.cpp](src/moveit_jetcobot_pkg/src/move_to_target_pose.cpp)

- Topics
    - /target_pose
    - /target_angles
- Msg type (for both)
    - std_msgs/msg/Float64MultiArray (explicitly 6 float data)

Test commands:
```sh 
ros2 topic pub /target_pose std_msgs/msg/Float64MultiArray "{layout: {}, data: [0.10, -0.16, 0.27, 0, -0.1, 0]}" --once
```
```sh
ros2 topic pub /target_angles std_msgs/msg/Float64MultiArray "{layout: {}, data: [0, 0, 0, 0, 0, 0]}" --once
```