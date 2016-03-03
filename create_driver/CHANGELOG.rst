^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package create_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
