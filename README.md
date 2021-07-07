# Home Service Robot

Final project for Udacity's Robotics Software Engineer Nanodegree Program

<p align="center"><img src="./result/final.gif"></p>



**Project Goals**

The goal of this project was to design a robot's environment in gazebo and program the home-service robot that will map it's environment and autonomously navigate to pre-specified pickup and drop-off locations. For this one needed to:

* Design robot's environment with the Building Editor in Gazebo.
* Teleoperate the robot and manually test SLAM.
* Use the ROS navigation stack and manually command the robot using the 2D Nav Goal arrow in rviz to move to 2 different desired positions and orientations.
* Write a pick_objects node that commands the robot to move to the desired pickup and drop off zones.
* Write an add_markers node that subscribes to the robot odometry and publishes pick-up and drop-off markers to rviz.
* modify pick_objects node and add_markers node to establish communication between them, to complete desired home service robot implementation



### Directory Tree and contents
This directory represents the main project's `src` folder structure with following contents

* README.md: this file.
* **images** - folder with images and videos for this report
* **add_markers** - add marker C++ node
* **config** - folder with configuration file to specify pick-up and drop-off locations
* **map** - map and gazebo world files
* **pick_objects** - pick-objects C++ node
* **rvizConfig** - folder with rViz configurations used with some launch scripts
* **scripts** - shell scripts
	* `add_marker.sh` - script for testing add_marker concept with `add_markers_test.cpp`
	* `home_service.sh` - main script for the home-service-robot
	* `pick_objects.sh` - script for testing pick_objects concept with `pick_objects_test`
	* `test_navigation.sh` - script for testing navigation
	* `test_slam.sh` - script for performing SLAM and preparing map
* **slam_gmapping** - official ROS package with `gmapping_demo.launch` file
* **turtlebot** - official ROS package with `keyboard_teleop.launch` file
* **turtlebot_interactions** - official ROS package with `view_navigation.launch` file
* **turtlebot_simulator** - official ROS package with `turtlebot_world.launch` file

---

### Clone and Build

Since the folder presented here comprises only of ROS package, one needs to first create a catkin workspace and initialize it. Also, note that the official ROS packaged are already included here, but their dependencies need to be installed; steps for this are given below.

Within your `home` directory, execute the following:

```
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
```

Within `~/catkin_ws/src/` download or clone folders of this repository:

```
cd ~/catkin_ws/src/
git clone https://github.com/viks8dm/home-service-robot.git
```

Install dependencies:

```
rosdep -i install gmapping -y
rosdep -i install turtlebot_teleop -y
rosdep -i install turtlebot_rviz_launchers -y
rosdep -i install turtlebot_gazebo -y
```

`NOTE`: If any of the official packages give error, I recommed you delete associated folder and clone with src folder using appropriate line from here:

```
git clone https://github.com/ros-perception/slam_gmapping.git  
git clone https://github.com/turtlebot/turtlebot.git  
git clone https://github.com/turtlebot/turtlebot_interactions.git  
git clone https://github.com/turtlebot/turtlebot_simulator.git
```

Go back to catkin workspace and build it

```
cd ~/catkin_ws/
catkin_make
```

### Launch specific application and visualize

Specific applications can be launched using scripts provided. In this section I will go over how I have used these scripts.

##### Gazebo-world, SLAM test & map-creation

For my world, I made a new world where I made a small world to map it faster with a lot of features inside and use test_slam.sh to map it.

For SLAM-test go to `src/scripts` folder and run `test_slam.sh` script:

```
cd ~/catkin_ws/src/scripts
./test_slam.sh
```

This will launch:

* `turtlebot_world.launch` to deploy turtlebot in my world with specific pose
* `gmapping_demo.launch` to perform SLAM
* `view_navigation.launch` to observe map in rviz
* `keyboard_teleop.launch` to manually control robot

After navigating the robot around multiple times, once I was satisfied with the map appearance in comparison to the world-file, I saved it using:

`rosrun map_server map_saver -f <map-location-and-name>`

The corresponding map looks as follows:

![saved_map](./map/verysimplemap.jpeg)
*Fig.3: saved-map*

##### Localization and navigation test

For localization testing, I used `test_localization.sh` as follows:

```
cd ~/catkin_ws/src/scripts
./test_localization.sh
```

This will launch:

* `turtlebot_world.launch` to deploy turtlebot in my world with specific pose
* `amcl_demo.launch` to localize turtlebot
* `view_navigation.launch` to observe map in rviz

I used `2D Nav` tab in rviz to manually point out to two different goals, one at a time, and direct the robot to reach them and orient itself with respect to them. A sample image is shown below and the sample video can be found [here](https://youtu.be/DN2bRCeoVyI)

![nav_sample](./images/nav_sample.jpg)
*Fig.4: navigation-test sample*

##### Navigation Goal Node (pick-objects)

To test robot's capability to reach multiple goals, as specified by the program (and not manually), I created pick_objects package and specifically `pick_objects_test.cpp` function. This can be tested following script which launches turtlebot, AMCL, rviz and pick_objects node:

```
cd ~/catkin_ws/src/scripts
./pick_objects.sh
```

##### Virtual Objects Node (add-markers)

To model a virtual object with markers in rviz, I created add_markers package and specifically `add_markers_test.cpp` function. This can be tested following script which launches turtlebot, AMCL, rviz and add_markers node:

```
cd ~/catkin_ws/src/scripts
./add_markers.sh
```



##### Home-Service-Robot package

To simulate a full home service robot capable of navigating to pick up and deliver virtual objects, communication was established between the add_markers and pick_objects nodes via a `"/pickstatus"` topic. For this purpose modified versions of previous test codes were created respectively `pick_objects.cpp` and `add_markers.cpp`. The entire package can be launched using:

```
cd ~/catkin_ws/src/scripts
./home_service.sh
```


