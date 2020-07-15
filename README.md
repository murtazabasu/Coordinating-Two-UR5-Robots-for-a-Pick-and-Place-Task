# Coordinating Two UR5 Robots for a pick and place task
<p align="center">
<img src="https://github.com/murtazabasu/Coordinating-Two-UR5-Robots-for-a-Pick-and-Place-Task/blob/master/media/ur5_coord.gif" width="480">

This repository shows the coordination between two UR5 robots in ROS and Gazebo where one is the master and the other is the slave. The project is inspired from the works of [__`GitHub: lihuang3/ur5_ROS-Gazebo`__](https://github.com/lihuang3/ur5_ROS-Gazebo.git). Both the master and the slave uses the Robotiq85 gripper for grasping and are attached with a camera sensor for detecting the objects. The master robot detects the object placed at the starting position using the camera and brings it close to the slave robot. The slave robot detects the object from the master's gripper and grasps it. 
Finally the slave robot brings the object at the final position. The master and slave robot performs three functionalities:

1. Object detection using the camera sensor ([`ur5_vision_master.py`](https://github.com/murtazabasu/Coordinating-Two-UR5-Robots-for-a-Pick-and-Place-Task/ur5_notebook/ur5_vision_master.py)) and ([`ur5_vision_slave.py`](https://github.com/murtazabasu/Coordinating-Two-UR5-Robots-for-a-Pick-and-Place-Task/ur5_notebook/ur5_vision_slave.py))

2. Motion planning using python moveit_commander interface ([`motion_planning_master.py`](https://github.com/murtazabasu/Coordinating-Two-UR5-Robots-for-a-Pick-and-Place-Task/ur5_notebook/motion_planning_master.py)) and ([`motion_planning_slave.py`](https://github.com/murtazabasu/Coordinating-Two-UR5-Robots-for-a-Pick-and-Place-Task/ur5_notebook/motion_planning_slave.py)). 

3. Grasping action using the GripperActionController ([`send_gripper_master.py`](https://github.com/murtazabasu/Coordinating-Two-UR5-Robots-for-a-Pick-and-Place-Task/ur5_notebook/send_gripper_master.py)) and ([`send_gripper_slave.py`](https://github.com/murtazabasu/Coordinating-Two-UR5-Robots-for-a-Pick-and-Place-Task/ur5_notebook/send_gripper_slave.py))


- Video demo:
  [`Simulation video on Youtube`](https://www.youtube.com/watch?v=n6Vk9lIxKkg)

- How to cite this repository: 
  ```
    Khuzema Basuwala, M, Coordinating two UR5 robots for a pick and place with Robotiq85 Grippers, 
    (2020), GitHub repository, https://github.com/murtazabasu/Coordinating-Two-UR5-Robots-for-a-Pick-and-Place-Task.git
  
  ```


#### Criterion for using this repository:
- This project was tested on Ubuntu 18.04 with ROS Melodic.
- Make sure you have installed Python2.7 and some useful libraries/packages, such as Numpy, cv2, etc.
- Install ROS Melodic, Gazebo, universal robot, Moveit, RViz. 
- This repo was created with the workspace name as `catkin_ws3/src`
- Assuming your workspace is named as `catkin_ws3`, download the repository to `catkin_ws3/src/`
  ```
  $ cd catkin_ws3/src
  $ git clone https://github.com/murtazabasu/Coordinating-Two-UR5-Robots-for-a-Pick-and-Place-Task.git
  ```
Under the `catkin_ws3/src` there are three directories: 
 1. `robotiq` for the robotiq85 gripper
 2. `univesal_robot` for the ur5 robot models and moveit configuration files
 3. `ur5_notebook` for the launch files and the python scripts

- Build the code under directory `catkin_ws3/`,
  ```
  $ catkin_make
  $ source devel/setup.bash  
  ```
- Run the code with ROS and Gazebo
  ```
  $ roslaunch ur5_notebook initialize.launch 
  ```
  The motion planning and the gripper nodes for both the master and the slave can be initialized in the above launch file itself (uncomment the lines at the bottom) or each script can be made to run individually on seperate terminals.  

#### References
- [__`GitHub: lihuang3/ur5_ROS-Gazebo`__](https://github.com/lihuang3/ur5_ROS-Gazebo.git) Implementation of UR5 pick and place in ROS-Gazebo with a USB cam and vacuum grippers.
- [__`ROS UR industrial`__](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial)
- [__`ROS modern driver`__](https://github.com/iron-ox/ur_modern_driver)
- [__`ur_hardware_interface.cpp`__](https://github.com/iron-ox/ur_modern_driver/blob/883070d0b6c0c32b78bb1ca7155b8f3a1ead416c/src/ur_hardware_interface.cpp) 



