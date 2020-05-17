# Map My World
SLAM operation on a mobile robot using  the [rtabmap_ros](http://wiki.ros.org/rtabmap_ros) package from ROS.

### Github Repsitory Link
https://github.com/cynepton/mapmyworld

### Database Link to generated DB
https://drive.google.com/file/d/1WmD10BzOAxFMnnJDwFdVZdZzSDvcHr2b/view?usp=sharing

## Working with this project
### Requirements
- ROS (It comes with Gazebo installed)
  - For Ubuntu 16.04, install [ROS Kinetic Kame](http://wiki.ros.org/kinetic/Installation/Ubuntu)
  - For Ubuntu 18.04, install [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation)

1. Clone the repository:
    ```
    git clone -b master https://github.com/cynepton/mapmyworld.git
    ```
2. Navigate to the `catkin_mapmyworld` folder and run:
    ```
    catkin_make
    ```
3. First, launch the Gazebo world and RViz, spawn the robot in the environment:
    ```
    roslaunch <YOUR PACKAGE NAME> world.launch
    ```
4. Then, launch the `teleop` node:
    ```
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```
5. Finally, launch your mapping node:
    ```
    roslaunch <YOUR PACKAGE NAME> mapping.launch
    ```

6. Navigate the robot in the simulation to create map for the environment! When you are all set, terminal the node and you could find your map `db` file in the place you specified in the `launch` file. If you did not modify the argument, it will be located in the `/root/.ros/` folder.