^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package create_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.0 (2023-05-21)
------------------

3.0.0 (2023-05-13)
------------------
* Add execution time dependency on launch_xml
* Update maintainer email
* Update package.xml descriptions
* Fixed left/right wheel joints in gazebo diff drive plugin parameters (`#77 <https://github.com/autonomylab/create_robot/issues/77>`_) (`#81 <https://github.com/autonomylab/create_robot/issues/81>`_)
* Port to ROS 2 (foxy) (`#8 <https://github.com/autonomylab/create_robot/issues/8>`_)
* Fix xacro warning
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
* Contributors: Jacob Perron, Pedro Grojsgold

1.3.0 (2018-06-10)
------------------
* Migrate to package.xml format 2
    * Minor linting to package files.
* Update install rules
* Refactor launch files and expose robot base and odometry frame IDs as parameters
* Refactor CMakeLists.txt and package.xml files and add missing install rules
* Contributors: Jacob Perron

1.2.0 (2016-10-07)
------------------
* Add support for more than two robot models and fix compiling with latest changes from libcreate.
* Contributors: Ben Wolsieffer

1.1.0 (2016-07-23)
------------------
* Refactor URDF to support differences between Create 1 and 2, while using a common base.
* Include a mesh for the Create 1, borrowed from turtlebot_create.
* Update launch files to support Create 1.
* Contributors: Ben Wolsieffer

1.0.1 (2016-05-24)
------------------

1.0.0 (2016-04-01)
------------------
* Add ca_description package
* Contributors: Jacob Perron
