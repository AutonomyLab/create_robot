# create_autonomy

[ROS](http://ros.org) driver for iRobot's [Create 1 & 2](http://www.irobot.com/About-iRobot/STEM/Create-2.aspx).

* Documentation: TODO
* ROS wiki page: TODO [](http://wiki.ros.org/create_autonomy)
* Code API: TODO 
* Author: [Jacob Perron](http://jacobperron.ca) ([Autonomy Lab](http://autonomylab.org), [Simon Fraser University](http://www.sfu.ca))

## Build Status
* TravisCI (Ubuntu _Trusty_, ROS _Indigo_ and _Jade_) _   _ ![Build Status](https://api.travis-ci.org/AutonomyLab/create_autonomy.svg?branch=master)

## Features

|  Feature      |  Status       |
|---------------|---------------|
|  Odometry     | Available     |
|  Safe mode    | Planned       |
|  Clean demo   | N/A       |    
|  Dock demo    | Available     |
|  Drive wheels | Available     |
|  Drive (v,w)  | Available     |
|  Brush motors | Planned       |
|  LEDs         | Available     |
|  Digit LEDs   | Available     |
|  Sound        | Planned #5      |
|  Wheeldrop    | Planned #6      |
|  Bumpers      | Planned #6      |
|  Cliff sensor | Planned       |
|  Dirt detect  | N/A       |    
|Omni IR sensor | Available     |
| Left IR sensor| N/A       |    
|Right IR sensor| N/A       |    
|  Battery info | Available     |
|  Light sensors| Planned       |
|Create 1 support| Developing #1   |
| **_Diagnostics_** |           |
|Corrupt Packets| Planned       |
| Phyiscal tests| Planned       |



## Install

#### Prerequisites 

* [ROS](http://wiki.ros.org/ROS/Installation) _Indigo_ or _Jade_
* Ubuntu packages: `python-rosdep`, `python-catkin-tools`

```bash
$ sudo apt-get install python-rosdep python-catkin-tools`
```

#### Compiling

1. Create a catkin workspace
``` bash
$ mkdir -p create_ws/src
$ cd create_ws
$ catkin init
```
2. Clone this repo
```bash
$ git clone https://github.com/AutonomyLab/create_autonomy.git
```
3. Install dependencies
```bash
$ rosdep update
$ rosdep install --from-paths src -i
```
4. Build
```bash
$ catkin build
```
#### USB Permissions
5. In order to connect to Create over USB, ensure your user is in the dialout group
```bash
$ sudo usermod -a -G dialout $USER
```
6. Logout and login for permission to take effect

## Running create_driver

### Setup

Connect computer to Create's 7-pin serial port
  - If using Create 1, ensure that nothing is connected to Create's DB-25 port

### Launch file

```bash
$ roslaunch create_driver create_driver create.launch [create_1:=false]
```

### Parameters
TODO

### Publishers
TODO

### Subscribers
TODO

