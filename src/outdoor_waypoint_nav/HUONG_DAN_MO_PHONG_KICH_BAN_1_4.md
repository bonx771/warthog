# Hướng dẫn mô phỏng Kịch bản 1 đến 4

Tài liệu này dùng workspace `/home/bo/warthog` và tách theo đúng hai
terminal: Terminal 1 chạy Gazebo + localization/navigation; Terminal 2 chạy
**một** kịch bản. Không chạy thêm `joy_launch_control_sim.launch` khi đã chạy
một launch kịch bản, vì launch kịch bản đã có control phù hợp. Cả K1–K4 mặc
định dùng cùng field trống 1 km `worlds/scenario_4_open.world`, không có tường
hay vật cản.

Trước khi chạy lần đầu:

```bash
source /opt/ros/noetic/setup.bash
source /home/bo/warthog/devel/setup.bash
```

`outdoor_waypoint_nav_sim.launch` mặc định chỉ mở RViz. Gazebo GUI bị tắt,
nhưng `gzserver` vẫn chạy nền để mô phỏng chuyển động, GPS, IMU, encoder và
`/clock`.

## Luồng GNSS chung

Cả K1–K4 tắt chuyển waypoint sớm theo bán kính
(`waypoint_advance_radius=0`). Direct-pursuit dùng ngưỡng xác nhận 0,05 m, là
mức tối thiểu C++ cho phép nếu không sửa code.

Gazebo phát GPS sạch ở `/gps/fix`. Terminal 2 là publisher duy nhất của
`/outdoor_waypoint_nav/gps/fix_selected`; Terminal 1 nhận đúng topic này trước
khi đưa qua `gps_covariance_relay`, `navsat_transform` và EKF2.

| Kịch bản | Luồng vào EKF2 |
| --- | --- |
| K1 | `/gps/fix` → relay → `gps/fix_selected` |
| K2 | `/gps/fix` → `gnss_correlated_noise` → `/gps/fix_noisy` → relay → `gps/fix_selected` |
| K3 | `/gps/fix` → `gnss_outage_gate` → `/gps/fix_off` → relay → `gps/fix_selected` |
| K4 | `/gps/fix` → relay → `gps/fix_selected`; manager chỉ đọc `/gps/fix` để lấy WP1 |

K2 giữ covariance tăng thêm của nhiễu khi vào EKF. K3 đo khoảng GNSS-off từ
`/outdoor_waypoint_nav/odometry/filtered`, đúng với EKF odometry có trong mô
phỏng.

## Kịch bản 1 — GPS bình thường

Terminal 1:

```bash
roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_sim.launch
```

Terminal 2:

```bash
roslaunch outdoor_waypoint_nav scenario_1_gnss_normal_sim.launch
```

Đợi GPS/EKF ổn định, sau đó bấm `r`/RB. Kịch bản dùng
`waypoint_files/points_sim.txt` và direct-pursuit như bản chạy thực.

## Kịch bản 2 — Nhiễu GNSS tương quan

Terminal 1:

```bash
roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_sim.launch
```

Terminal 2, ví dụ mức nhiễu nhẹ:

```bash
roslaunch outdoor_waypoint_nav scenario_2_gnss_noise_sim.launch \
  innovation_sigma_m:=3.0
```

Nhiễu mặc định bật sau khi đạt WP2 và tắt khi đạt WP4. Có thể thay `3.0` bằng
`6.0` hoặc `9.0`. Kiểm tra nhanh bằng:

```bash
rostopic echo /outdoor_waypoint_nav/gnss_noise/active
rostopic echo /gps/fix_noisy
```

## Kịch bản 3 — Mất GNSS tạm thời

Terminal 1:

```bash
roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_sim.launch
```

Terminal 2:

```bash
roslaunch outdoor_waypoint_nav scenario_3_gnss_outage_sim.launch \
  outage_distance_m:=20.0
```

Mặc định GNSS bị ngắt sau WP2, giữ im trong 20 m odometry, rồi tự bật lại. Khi
đang ngắt, `/gps/fix_off` và `gps/fix_selected` không có message mới; đó là
hành vi chủ đích để `navsat_transform` thấy mất GNSS thực sự.

## Kịch bản 4 — Tuyến sinh tự động theo góc

K4 dùng field trống 1 km mặc định giống K1–K3, đủ cho tuyến dài 300 m.

Terminal 1:

```bash
roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_sim.launch
```

Terminal 2, ví dụ góc 15 độ:

```bash
roslaunch outdoor_waypoint_nav scenario_4_trajectory_sim.launch theta_deg:=15
```

Trình tự phím giữ nguyên bản chạy thực:

1. `l`: chuẩn bị lấy WP1.
2. Đặt xe đứng yên tại WP1, hướng theo trục giữa, rồi nhấn `y`; node lấy mẫu
   GPS và heading trong 5 giây.
3. `c`: sinh 5 waypoint theo góc 15/30/45 độ, lưu
   `waypoint_files/points_scenario_4_sim.txt` và metadata JSON.
4. `k`: kiểm tra, khóa đủ 5 waypoint.
5. `r`: chạy direct-pursuit qua các waypoint đó.

`b` dừng khẩn. Sau một run K4, khởi động lại Terminal 2 trước run tiếp theo để
PNG/CSV không bị lẫn dữ liệu. Có thể dùng `route_length_m:=...` nhỏ hơn chỉ để
debug, nhưng kết quả K4 so sánh chính thức nên giữ `300.0` m.

## Kết quả và cách chạy cũ

Mỗi scenario tự lưu cả PNG và CSV vào cùng một thư mục run ngay trong
`results/simulation/` (không qua tầng `experiment_runs`). Thư mục run được đánh số riêng theo
từng scenario, ví dụ `scenario_1_gnss_normal_sim_1`, `_2`, `_3`; các thư mục
timestamp cũ được bỏ qua khi tính số thứ tự. Mỗi run chỉ tạo
`waypoints.csv`, `reference_gnss.csv`, `controller_trajectory.csv` và
`position_error.png`. Cross-track RMSE được in ở Terminal 2 và phát trên topic
`/outdoor_waypoint_nav/experiment_logger/cross_track_rmse_m`.

Cặp lệnh mô phỏng cũ vẫn chạy được:

```bash
roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_sim.launch
roslaunch outdoor_waypoint_nav joy_launch_control_sim.launch
```

`joy_launch_control_sim.launch` giờ tự relay GPS sạch sang `gps/fix_selected`
để tương thích với topology mới. Chỉ dùng cặp này khi không chạy K1–K4; không
khởi động nó song song với scenario launch.
