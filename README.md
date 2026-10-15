# norlab_imu_tools

Gyro bias estimation for a stationary start, used on every IMU of the robot (`norlab_robot` launches both nodes in `mti100.launch.py` and `rsairy.launch.py`).

- `imu_bias_observer`: averages the gyro on `imu_topic_in` over `target_observation_samples` (default 4000) and publishes the bias once on `bias_topic_out`. Keep the robot still while it runs.
- `imu_bias_compensator_node`: subtracts the latest bias from `bias_topic_in` from every message on `imu_topic_in` and republishes it on `imu_topic_out`.
