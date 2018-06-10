^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ca_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
