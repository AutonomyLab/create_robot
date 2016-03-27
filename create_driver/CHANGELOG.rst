^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package create_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
