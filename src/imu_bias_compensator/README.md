# Bước 1 – Chạy calibrate lấy bias:
bashroslaunch imu_bias_compensator calibrate.launch
roslaunch imu_bias_compensator calibrate.launch duration:=90
hăoj
rosrun imu_bias_compensator gyro_calibrator.py _duration:=90

# Bước 2 – Điền giá trị vào config/gyro_bias_static.yaml: Tự động

gyro_bias_x: 0.000312
gyro_bias_y: -0.000145
gyro_bias_z: 0.001823   # ← quan trọng nhất

# Chạy republisher:
roslaunch imu_bias_compensator imu_republisher.launch

# So sánh trước/sau
rostopic echo /imu/data_raw | grep -A3 angular_velocity
rostopic echo /imu/data_raw_bias | grep -A3 angular_velocity

# Xem gz khi đứng yên có về 0 không
rostopic echo /imu/data_raw_bias/angular_velocity/z


# 2. Calibrate lần đầu (robot đứng yên)
roslaunch imu_bias_compensator calibrate.launch
# → tự ghi config/gyro_bias.yaml

# 3. Chạy bình thường
roslaunch imu_bias_compensator zupt_compensator.launch

# Sau đó trỏ Madgwick sang topic mới:
yamlimu_topic: /imu/data_corrected
use_mag: true

# Theo dõi bias real-time:
rostopic echo /imu/bias_status
# [bias_x, bias_y, bias_z, is_static, static_counter]
