# warthog_navigation

The **warthog_navigation** provides a [ROS] common packages for use slam with warthog robot.

**Keywords:** ROS, Slam, Navigation, Localization, Mapping

**Authors: [Matheus França](https://github.com/MatheusFranca-dev)<br />
Affiliation: [BIR - Brazilian Institute of Robotics](https://github.com/Brazilian-Institute-of-Robotics)<br />
Maintainer: Matheus França, br_matheus@hotmail.com**

## **Purpose of the Project**

Using warthog robot for navigation, localization and mapping, to develop and train slam techniques. All this with ROS integration.

### Supported Versions

- **Noetic**: Built and tested under [ROS] Noetic and Ubuntu 20.04

### Dependencies 
- [ROS] : An open-source robot framework. (Version == Noetic)
- [warthog](https://www.clearpathrobotics.com/assets/guides/kinetic/warthog/WarthogInstallation.html)
- [velodyne_simulator](https://bitbucket.org/DataspeedInc/velodyne_simulator/src/master/)
- [LMS1xx](https://github.com/clearpathrobotics/lms1xx)
- [rf2o_laser_odometry](https://github.com/tianb03/rf2o_laser_odometry)

# **Table of Contents**
- [**warthog_navigation**](#warthog_navigation)
- [**Purpose of the Project**](#purpose-of-the-project)
    - [Supported Versions](#supported-versions)
    - [Dependencies](#dependencies)
- [**File System**](#file-system)
- [**Installation**](#installation)
	- [Building from Source:](#building-from-source)
	- [Example of Usage](#example-of-usage)
- [**License**](#license)
- [**Bugs & Feature Requests**](#bugs--feature-requests)

# **File System**
- [warthog_simulation](https://github.com/MatheusFranca-dev/warthog_navigation/tree/main/warthog_simulation) : Define the gazebo package for simulate the robot and the world objects. 
- [warthog_slam](https://github.com/MatheusFranca-dev/warthog_navigation/tree/main/warthog_slam) : Contains the slam parameter.

# **Installation**

###  Building from Source:

Attention, if you haven't installed [ROS] yet, please check [Ubuntu install of ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu). Desktop-Full Install is the recommended one in order to work with this repository.  

**Building:**

First, lets create a catkin workspace.

    $ mkdir -p ~/catkin_ws/src

Then, clone **warthog_navigation** inside your workspace source.

	$ git clone https://github.com/MatheusFranca-dev/warthog_navigation.git

Don't forget to also clone the [Dependencies](#dependencies) in your source folder.

Now, just build your catkin workspace.

    $ cd ~/catkin_ws
    $ catkin_make

Don't forget to source your workspace before using it.
    
    $ source devel/setup.bash

## Example of Usage

For simulate the models, Run:

	$ roslaunch warthog_simulation warthog_gazebo.launch rviz:=true config:=localization

Check the config parameter (It's a rviz configuration):
    
 - navigation: For navigate without a map
 - gmapping: To create a map
 - localization: For navigate with a map
 - rtabmap: For slam with realsense sensor

**NAVIGATION WITHOUT A MAP**

    $ roslaunch warthog_slam odom_navigation.launch                   

**MAKING A MAP**
    
    $ roslaunch warthog_slam gmapping.launch         

When you’re satisfied, you can save the produced map using map_saver:

    $ rosrun map_server map_saver -f mymap

This will create a mymap.yaml and mymap.pgm file in your current directory.       

**NAVIGATION WITH A MAP**

    $ roslaunch warthog_slam amcl.launch map_file:=/path/to/my/map.yaml

![banner](https://github.com/MatheusFranca-dev/warthog_navigation/blob/main/doc_resources/v1-warthog-navigation-2021-04-01.gif)

**SLAM WITH RTABMAP**

For create a map, use:

    $ roslaunch warthog_slam warthog_rtabmap.launch localization:=false

After create a map, you could kill the application and run:

    $ roslaunch warthog_slam warthog_rtabmap.launch localization:=true

![banner](https://github.com/MatheusFranca-dev/warthog_navigation/blob/main/doc_resources/v1_rtabmap.png)

# **License**

Warthog Navigation Project source code is released under a [MIT License](/LICENSE).

# **Bugs & Feature Requests**

Please report bugs and request features using the [Issue Tracker].

<!-- Hyperlinks -->
[ROS]: https://www.ros.org
[Issue Tracker]: https://github.com/MatheusFranca-dev/warthog_navigation/issues