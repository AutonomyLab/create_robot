^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package create_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.0 (2023-05-21)
------------------
* Fix compilation for ROS Humble
* Contributors: Jacob Perron

3.0.0 (2023-05-13)
------------------
* Replace Travis CI with GitHub workflow and lint (`#102 <https://github.com/autonomylab/create_robot/issues/102>`_)
* Update maintainer email
* Add parameter to enable the OI mode reporting bug workaround (`#95 <https://github.com/autonomylab/create_robot/issues/95>`_)
* Add cliff sensors (`#93 <https://github.com/autonomylab/create_robot/issues/93>`_)
* Add motor control (`#97 <https://github.com/autonomylab/create_robot/issues/97>`_)
* Disable libcreate's signal handler (`#89 <https://github.com/autonomylab/create_robot/issues/89>`_)
* Use rclcpp::spin instead of own implementation.
* Use Node::now instead of rclcpp::Clock::now.
* Make CreateDriver derive from rclcpp::Node.
* Fix latching condition (`#72 <https://github.com/autonomylab/create_robot/issues/72>`_)
* Port to ROS 2 (foxy) (`#8 <https://github.com/autonomylab/create_robot/issues/8>`_)
* Stricter compile options
* Switch to tf2
* Reduce diagonal covariance values
* Remove old install commands
* Move launch and config files to create_bringup
* Update maintainer info
* Fix catkin lint error
* Rename packages for clarity
  Originally the packages were named as 'create_autonomy', with the 'ca\_' suffix for short.
  This was due to a name conflict with the turtlebot packages (e.g. create_driver and create_description):
  https://github.com/turtlebot/turtlebot_create.git
  Since the turtlebot packages are not released into ROS Melodic, the plan is to release these packages instead.
  If the turtlebot packages are eventually released, it should be straightforward to adapt them to use the
  description and driver packages here.
  Summary of renaming:
  * create_autonomy -> create_robot
  * ca_description -> create_description
  * ca_driver -> create_driver
  * ca_msgs -> create_msgs
  * ca_tools -> create_bringup
* Contributors: Jacob Perron, Josh Gadeken, Owen Hooper, Pedro Grojsgold, pgold

1.3.0 (2018-06-10)
------------------
* Add explicit dependency on catkin_EXPORTED_TARGETS
    * This ensures ca_msgs is built before ca_driver.
* Migrate to package.xml format 2
    * Minor linting to package files.
* Add roslint test and fix lint issues
* find_package libcreate instead of downloading as external project
* Add support for defining and playing songs
* Update install rules
* Refactor launch files and expose robot base and odometry frame IDs as parameters
* Refactor CMakeLists.txt and package.xml files and add missing install rules
* Contributors: Clyde McQueen, Jacob Perron

1.2.0 (2016-10-07)
------------------
* Add support for more than two robot models and fix compiling with latest changes from libcreate.
* Contributors: Ben Wolsieffer

1.1.0 (2016-07-23)
------------------
* Add diagnostics (Battery, Safety, Serial, Mode, Driver)
* Refactor URDF to support differences between Create 1 and 2, while using a common base.
* Include a mesh for the Create 1, borrowed from turtlebot_create.
* Update launch files to support Create 1.
* Publish JointState messages for the wheels.
* Contributors: Ben Wolsieffer, Jacob Perron

1.0.1 (2016-05-24)
------------------
* Get pose covariance from underlying library
* Make publishing TF (odom frame) optional
* Add launch file argument making description optional
* Update libcreate tag, with odometry patch
* Add git as build dependency
* Move ChargingState publisher with other battery state information
* Add Bumper message with contact sensor states and light sensor states
* Add header to all custom message types
* Contributors: Jacob Perron

1.0.0 (2016-04-01)
------------------
* Update libcreate git tag
* Rename variables according to ROS cpp style guide
* Run clang-format
* Fix typo
* Add ca_description as runtime dependency to ca_driver
* Change message type of various battery info topics
* Add ChargingState and Mode messages
* Add charging state and mode publishers
* Add 'ca_msgs' package
* Update ca_driver README.md
* Rename 'create_driver' and 'create_tools' to 'ca_driver' and 'ca_tools'
* Contributors: Jacob Perron

0.4.0 (2016-03-26)
------------------
* Add timestamp to odometry message
* Update libcreate tag (with odometry fix)
* Fix sign error for battery current and temperature
* Update tag for libcreate
* Add delay before starting dock demo in case of Create 1
* Compute and publish battery charge ratio
* Add README.md for create_driver
* Add support for Create 1
* Contributors: Jacob Perron

0.3.0 (2016-03-17)
------------------
* Add dock / undock support
* Publish characters received by omni directional IR sensor
* Add battery info support
* Contributors: Jacob Perron

0.2.0 (2016-03-03)
------------------
* Add covariances to odometry messages
* Add set 7Seg display with ASCII
* Add LED support
* Add publishers for button presses
* Contributors: Jacob Perron

0.1.0 (2016-02-05)
------------------
* Fixed bugs: Private nodehandle now gets params, added missing timestamp to tf messages
* Added tf broadcaster for odom frame
* Added CI (travis)
* Now publishing velocities in odom messages
* Added anti-latch mechanism
* Switch to using node handle with private namespace for parameters only
* Velocity commands now accepted in m/s
* Updated launch file
* Initial commit
* Contributors: Jacob Perron
