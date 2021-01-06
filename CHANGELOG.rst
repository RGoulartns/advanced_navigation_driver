^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package advanced_navigation_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2020-01-06)
-------------------
* Added orientation and velocities covariances to the imu message
* Added a loop rate
* Added a launch file
* Converted orientation, velocities and acceleration to ENU (instead of NED)
* Fixed a bug where the driver would publish the same message twice per loop
* Optional: Replaced INS reported timestamp to ros::time::now() timestamp

