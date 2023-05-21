^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package create_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.0 (2023-05-21)
------------------

3.0.0 (2023-05-13)
------------------
* Update maintainer email
* Update package.xml descriptions
* Add cliff sensors (`#93 <https://github.com/autonomylab/create_robot/issues/93>`_)
* Add motor control (`#97 <https://github.com/autonomylab/create_robot/issues/97>`_)
* Port to ROS 2 (foxy) (`#8 <https://github.com/autonomylab/create_robot/issues/8>`_)
* Update maintainer info
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
* Contributors: Jacob Perron, Owen Hooper, Pedro Grojsgold

1.3.0 (2018-06-10)
------------------
* Migrate to package.xml format 2
    * Minor linting to package files.
* Add support for defining and playing songs
* Refactor CMakeLists.txt and package.xml files and add missing install rules
* Contributors: Clyde McQueen, Jacob Perron

1.2.0 (2016-10-07)
------------------

1.1.0 (2016-07-23)
------------------

1.0.1 (2016-05-24)
------------------
* Add Bumper message with contact sensor states and light sensor states
* Add header to all custom message types
* Contributors: Jacob Perron

1.0.0 (2016-04-01)
------------------
* Add ChargingState and Mode messages
* Add charging state and mode publishers
* Add 'ca_msgs' package
* Contributors: Jacob Perron
