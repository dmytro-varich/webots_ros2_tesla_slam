^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package webots_ros2_tesla_slam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2026.1.0 (2026-06-11)
------------------
* Split launch setup into dedicated simulation, SLAM, and Navigation2 entry points.
* Added selectable Navigation2 localization modes: GPS, AMCL, and odometry-only.
* Added GPS, IMU, NavSat relay, and dual-EKF GPS localization support.
* Added a front/rear laser scan merger for AMCL localization.
* Updated Cartographer configuration for front and rear lidar scans.
* Updated Webots world obstacles, sensors, and map assets.
* Tuned Nav2 planning, control, costmaps, and Ackermann command conversion.
* Improved odometry, IMU, and TF publishing for the Tesla driver.

2026.0.0 (2026-05-15)
------------------
* Based on the webots_ros2_tesla package with project-specific modifications.
* Added SLAM and Nav2 functionality.
* Added configuration, maps, and behavior tree files.
* Updated launch, resource, and world files.
* Updated lane_follower and tesla_driver nodes to support SLAM and Nav2.

2023.1.0 (2023-06-29)
------------------
* Clean simulation reset in launch file.
* Update driver node to new WebotsController node.

2023.0.2 (2023-02-07)
------------------
* Updated supervisor launch.

2022.1.3 (2022-11-02)
------------------
* Added macOS support.
* Added reset handler to support simulation reset from Webots.

2022.1.2 (2022-10-21)
------------------
* Added WSL support.

1.2.3 (2022-06-01)
------------------
* Fixed support for Humble and Rolling.

1.2.0 (2021-12-21)
------------------
* Adapt the worlds to the new R2022a FLU convention.

1.1.2 (2021-11-03)
------------------
* Utilize the 'webots_ros2_driver' instead of 'webots_ros2_core'.
* Added code compliance for 'ROS Foxy'.

1.0.6 (2021-04-10)
------------------
* Initial version
