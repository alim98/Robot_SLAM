Problem Statement

In this problem, you are asked to perform the following tasks for a mobile robot.
This robot is equipped with IMU sensors (with a sampling rate of about 100 samples per second) to measure its position at each moment, and also with a Lidar sensor (with a sampling rate of about 10 samples per second) to measure its distance from walls. The information from these sensors contains noise and bias. Moreover, instead of raw IMU data, an algorithm already processes this information and reports the robot’s position in terms of x, y, θ at each moment.

Note that, due to the nature of the IMU sensor, its information contains cumulative error, meaning that the error increases over time. However, the Lidar sensor does not have this issue. Therefore, the estimated robot position (x, y, θ) compared to the true position of the robot increases in error as time progresses.

Extract the Lidar sensor model (Sensor Model) using any desired method

Extract the robot motion model (Motion Model) using any desired method

Perform simultaneous localization and mapping (SLAM) using the Graph Optimization method

Important note: 
You cannot use libraries like SLAM and Graph Optimization. You should implement the algorithms yourself. 