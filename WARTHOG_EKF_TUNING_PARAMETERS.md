# Bang tham so tune Warthog EKF simulation

Dung cho hai lenh:

```bash
cd ~/warthog
roslaunch outdoor_waypoint_nav outdoor_waypoint_nav_sim.launch
roslaunch outdoor_waypoint_nav joy_launch_control_sim.launch
```

Luồng chính:

1. `outdoor_waypoint_nav_sim.launch` chạy Gazebo, EKF/navsat, RViz, `move_base`, `safety_node`.
2. `joy_launch_control_sim.launch` nghe joystick/keyboard. Khi nhấn nút gửi waypoint, nó chạy `send_goals_sim.launch`.
3. `send_goals_sim.launch` chạy node `gps_waypoint`, đọc waypoint rồi gửi goal tới `/move_base`.
4. `move_base` dùng `NavfnROS` cho global path và `TrajectoryPlannerROS` cho local path.
5. Lệnh vận tốc đi theo luồng: `move_base -> /cmd_vel_intermediate -> safety_node -> /cmd_vel`.

Ghi chú:

- `warthog` hiện dùng `base_local_planner/TrajectoryPlannerROS`, không dùng MPC/TEB.
- `move_base_gps_sim.launch` load `base_local_planner_params.yaml` rồi override nhiều tham số ngay trong launch. Khi cùng tham số xuất hiện ở cả hai chỗ, giá trị trong launch thường thắng.
- Sau khi sửa launch/YAML, restart lại launch để ROS nạp tham số mới.

## 1. Waypoint, đổi mục tiêu, gửi lại goal

| Muốn chỉnh hành vi | Tham số | Giá trị hiện tại | File | Tăng lên thì sao | Giảm xuống thì sao |
|---|---:|---:|---|---|---|
| File waypoint dùng trong sim | `/outdoor_waypoint_nav/coordinates_file` | `/waypoint_files/points_sim.txt` | `src/outdoor_waypoint_nav/launch/include/send_goals_sim.launch` | Dùng file waypoint khác nếu đổi path | Sai path thì không gửi waypoint được |
| Frame gửi goal cho `move_base` | `/outdoor_waypoint_nav/goal_frame` | `map` | `send_goals_sim.launch` | Thường không đổi | Sai frame làm goal lệch |
| Bán kính tự chuyển sang waypoint tiếp theo | `/outdoor_waypoint_nav/waypoint_advance_radius` | `1.0 m` | `send_goals_sim.launch` | Chuyển waypoint sớm hơn, ít đứng chờ | Đi sát waypoint hơn, dễ khựng nếu quá nhỏ |
| Khi gần waypoint thì nhìn trước về waypoint sau | `/outdoor_waypoint_nav/waypoint_heading_preview_radius` | `0.7 m` | `send_goals_sim.launch` | Chuẩn bị cua/chuyển hướng sớm hơn | Đổi hướng muộn, dễ cua gắt |
| Giới hạn góc goal mới so với hướng robot | `/outdoor_waypoint_nav/max_waypoint_goal_yaw_delta` | `0.35 rad` | `send_goals_sim.launch` | Goal đổi hướng mạnh hơn, có thể cua nhanh hơn | Mượt hơn nhưng căn hướng chậm hơn |
| Tự gửi lại goal hiện tại | `/outdoor_waypoint_nav/auto_replan_current_goal_enabled` | `true` | `send_goals_sim.launch` | Bật để cập nhật đường liên tục | Tắt thì ít reset goal hơn nhưng đường cập nhật chậm |
| Chu kỳ tự replan current goal | `/outdoor_waypoint_nav/auto_replan_current_goal_interval` | `0.5 s` | `send_goals_sim.launch` | Replan thưa hơn, đỡ giật CPU | Replan dày hơn, phản ứng nhanh nhưng dễ giật |
| Cho safety yêu cầu replan | `/outdoor_waypoint_nav/safety_replan_enabled` | `true` | `send_goals_sim.launch` | Dễ thoát kẹt khi safety thấy vấn đề | Ít replan bất ngờ hơn |
| Nghỉ giữa 2 lần safety replan | `/outdoor_waypoint_nav/safety_replan_cooldown` | `0.0 s` | `send_goals_sim.launch` | Tránh spam replan | Replan ngay khi có yêu cầu |

## 2. TrajectoryPlannerROS: tốc độ, lấy mẫu, bám path

| Muốn chỉnh hành vi | Tham số | Giá trị hiện tại | File | Tăng lên thì sao | Giảm xuống thì sao |
|---|---:|---:|---|---|---|
| Tốc độ tiến tối đa | `TrajectoryPlannerROS/max_vel_x` | `1.4 m/s` | `src/warthog_navigation/warthog_slam/launch/move_base_gps_sim.launch` | Robot chạy nhanh hơn nếu safety/costmap không kẹp | Chạy chậm hơn, dễ ổn định hơn |
| Tốc độ tiến tối thiểu khi chọn đi tiến | `TrajectoryPlannerROS/min_vel_x` | `0.35 m/s` | `move_base_gps_sim.launch` | Ít bò chậm, mạnh hơn | Có thể đi chậm/mềm hơn |
| Gia tốc tiến | `TrajectoryPlannerROS/acc_lim_x` | `4.0 m/s^2` | `move_base_gps_sim.launch` | Lên tốc nhanh hơn, có thể giật | Lên tốc mượt/chậm hơn |
| Tốc độ quay tối đa | `TrajectoryPlannerROS/max_vel_theta` | `1.0 rad/s` | `move_base_gps_sim.launch` | Quay/cua nhanh hơn | Cua chậm hơn |
| Tốc độ quay ngược tối đa | `TrajectoryPlannerROS/min_vel_theta` | `-1.0 rad/s` | `move_base_gps_sim.launch` | Cho quay phải nhanh hơn nếu âm lớn hơn về trị tuyệt đối | Quay phải chậm hơn |
| Tốc quay tại chỗ tối thiểu | `TrajectoryPlannerROS/min_in_place_vel_theta` | `0.30 rad/s` | `move_base_gps_sim.launch` | Không bị ì khi xoay | Xoay mềm hơn, có thể ì |
| Gia tốc góc | `TrajectoryPlannerROS/acc_lim_theta` | `2.5 rad/s^2` | `move_base_gps_sim.launch` | Đổi tốc quay nhanh hơn | Quay mượt hơn |
| Bán kính đạt goal XY | `TrajectoryPlannerROS/xy_goal_tolerance` | `0.4 m` | `src/warthog_navigation/warthog_slam/params/base_local_planner_params.yaml` | Dễ coi là tới waypoint hơn | Đi sát waypoint hơn, dễ dừng chờ |
| Sai số yaw goal được chấp nhận | `TrajectoryPlannerROS/yaw_goal_tolerance` | `3.14 rad` | `move_base_gps_sim.launch` | Hầu như không cần đúng hướng tại waypoint | Nếu giảm, robot phải xoay đúng hướng hơn |
| Giữ trạng thái đã đạt XY | `TrajectoryPlannerROS/latch_xy_goal_tolerance` | `true` | `move_base_gps_sim.launch` | Đỡ bị ra/vào vùng goal liên tục | Tắt dễ dao động quanh goal |
| Thời gian mô phỏng trajectory | `TrajectoryPlannerROS/sim_time` | `3.0 s` | `move_base_gps_sim.launch` | Nhìn xa hơn, tính nặng hơn | Nhìn ngắn hơn, phản ứng nhanh nhưng dễ cục bộ |
| Số mẫu tốc độ tiến | `TrajectoryPlannerROS/vx_samples` | `10` | `move_base_gps_sim.launch` | Nhiều lựa chọn tốc hơn, nặng hơn | Nhẹ hơn nhưng thô hơn |
| Số mẫu tốc độ quay | `TrajectoryPlannerROS/vtheta_samples` | `20` | `move_base_gps_sim.launch` | Nhiều lựa chọn góc hơn, nặng hơn | Nhẹ hơn nhưng dễ chọn góc kém |
| Trọng số bám global path | `TrajectoryPlannerROS/pdist_scale` | `0.9` | `move_base_gps_sim.launch` | Bám path sát hơn | Thoáng hơn, dễ cắt cua |
| Trọng số tiến tới goal | `TrajectoryPlannerROS/gdist_scale` | `1.2` | `move_base_gps_sim.launch` | Ưu tiên lao tới goal, nhanh hơn | Bám path/né vật cản nhiều hơn |
| Trọng số tránh vật cản | `TrajectoryPlannerROS/occdist_scale` | `0.45` | `move_base_gps_sim.launch` | Né xa hơn, có thể vòng hơn | Đi sát vật cản hơn |
| Chấm điểm theo hướng nhìn | `TrajectoryPlannerROS/heading_scoring` | `false` | `move_base_gps_sim.launch` | Nếu bật, planner ưu tiên heading | Tắt thì score theo path/goal/obstacle |
| Khoảng nhìn trước khi heading scoring | `TrajectoryPlannerROS/heading_lookahead` | `1.2 m` | `move_base_gps_sim.launch` | Dùng nếu bật heading scoring | Dùng nếu bật heading scoring |

## 3. Costmap, lidar, inflation

| Muốn chỉnh hành vi | Tham số | Giá trị hiện tại | File | Tăng lên thì sao | Giảm xuống thì sao |
|---|---:|---:|---|---|---|
| Lidar/costmap nhìn vật cản xa bao nhiêu | `obstacle_range` | `10.0 m` | `src/warthog_navigation/warthog_slam/params/costmap_common_params.yaml` | Nhận vật cản xa hơn | Chỉ thấy vật cản gần hơn |
| Khoảng raytrace xóa vùng trống | `raytrace_range` | `12.0 m` | `costmap_common_params.yaml` | Xóa obstacle cũ xa hơn | Có thể giữ obstacle cũ lâu hơn |
| Topic lidar | `obstacles_layer/scan/topic` | `front/scan`, remap `/front/scan` | `costmap_common_params.yaml`, `move_base_gps_sim.launch` | Thường không đổi | Sai topic thì costmap không thấy vật cản |
| Footprint robot cho costmap | `footprint` | `[[-0.85,-0.75],[-0.85,0.75],[0.85,0.75],[0.85,-0.75]]` | `costmap_common_params.yaml` | Nới ra an toàn hơn, khó đi hẹp hơn | Thu nhỏ dễ luồn nhưng dễ va |
| Padding footprint hiệu lực trong sim | `global/local_costmap/footprint_padding` | `0.02 m` | `move_base_gps_sim.launch` | Giữ xa vật cản hơn | Đi sát hơn |
| Inflation global quanh vật cản | `global_costmap/inflater_layer/inflation_radius` | `2.0 m` | `move_base_gps_sim.launch` | Global path né xa hơn | Global path sát hơn |
| Inflation local quanh vật cản | `local_costmap/inflater_layer/inflation_radius` | `1.0 m` | `move_base_gps_sim.launch` | Local planner né xa hơn | Local planner đi sát hơn |
| Độ dốc cost inflation | `*/inflater_layer/cost_scaling_factor` | `0.8` | `move_base_gps_sim.launch` | Cost giảm nhanh hơn, có thể đi sát hơn | Vùng cost cao lan rộng hơn, né xa hơn |
| Kích thước global costmap | `global_costmap/width`, `height` | `100 x 100 m` | `move_base_gps_sim.launch` | Vùng global rộng hơn, nặng hơn | Nhẹ hơn nhưng thấy ít hơn |
| Kích thước local costmap | `local_costmap/width`, `height` | `15 x 15 m` | `move_base_gps_sim.launch` | Local nhìn rộng hơn, nặng hơn | Nhẹ hơn nhưng né muộn |
| Tần số update local costmap | `local_costmap/update_frequency` | `10 Hz` | `move_base_gps_sim.launch` | Phản ứng nhanh hơn | Có thể né chậm hơn |
| Frame local costmap | `local_costmap/global_frame` | `odom` | `move_base_gps_sim.launch` | `odom` thường mượt cho local | Đổi sai frame dễ giật |

## 4. safety_node: chặn tốc, giảm tốc gần goal, quay trước khi chạy

| Muốn chỉnh hành vi | Tham số | Giá trị hiện tại | File | Tăng lên thì sao | Giảm xuống thì sao |
|---|---:|---:|---|---|---|
| Bật/tắt safety node | `/outdoor_waypoint_nav/safety_enabled` | `true` | `src/outdoor_waypoint_nav/launch/include/safety_node.launch` | Bật lớp chặn tốc cuối | Tắt thì không lọc lệnh safety |
| Bắt đầu giảm tốc khi có vật cản trước mặt | `/outdoor_waypoint_nav/safety_slowdown_distance` | `0.8 m` | `safety_node.launch` | Giảm tốc sớm hơn | Giảm tốc muộn hơn, dễ sát vật cản |
| Dừng tiến khi vật cản quá gần | `/outdoor_waypoint_nav/safety_stop_distance` | `0.35 m` | `safety_node.launch` | Dừng xa hơn | Dừng sát hơn |
| Nửa góc quét phía trước | `/outdoor_waypoint_nav/safety_sector_half_angle_deg` | `20 deg` | `safety_node.launch` | Kiểm tra vùng rộng hơn | Hẹp hơn, ít bị vật bên hông ảnh hưởng |
| Timeout lệnh vận tốc | `/outdoor_waypoint_nav/safety_command_timeout` | `0.5 s` | `safety_node.launch` | Cho mất lệnh lâu hơn trước khi dừng | Dừng nhanh hơn khi mất lệnh |
| Bắt đầu giảm tốc gần goal | `/outdoor_waypoint_nav/goal_slowdown_distance` | `3.0 m` | `safety_node.launch` | Giảm tốc sớm hơn, ít quá đà | Giữ tốc lâu hơn |
| Khoảng sát goal dùng tốc thấp nhất | `/outdoor_waypoint_nav/goal_slowdown_stop_distance` | `0.6 m` | `safety_node.launch` | Chạy chậm rộng hơn ở cuối | Chỉ chậm rất sát goal |
| Tốc tối đa khi sát goal | `/outdoor_waypoint_nav/goal_slowdown_min_forward_speed` | `0.20 m/s` | `safety_node.launch` | Vào waypoint nhanh hơn | Vào waypoint mềm/chậm hơn |
| Tốc tham chiếu ngoài vùng giảm tốc | `/outdoor_waypoint_nav/goal_slowdown_max_forward_speed` | `1.4 m/s` | `safety_node.launch` | Không kẹp dưới tốc planner | Nếu thấp hơn planner sẽ kẹp tốc |
| Bật đứng yên quay về waypoint mới | `/outdoor_waypoint_nav/waypoint_align_enabled` | `true` | `safety_node.launch` | Ít vừa chạy vừa xoay, đỡ lượn sóng | Chuyển waypoint liên tục hơn nhưng dễ ngoằn |
| Sai số bắt đầu align | `/outdoor_waypoint_nav/waypoint_align_angle_threshold` | `0.35 rad` | `safety_node.launch` | Chỉ align khi lệch nhiều hơn | Dễ kích hoạt align hơn |
| Sai số để nhả chạy | `/outdoor_waypoint_nav/waypoint_align_release_angle` | `0.12 rad` | `safety_node.launch` | Nhả sớm hơn | Căn hướng kỹ hơn, có thể chậm |
| Goal gần hơn thì bỏ qua align | `/outdoor_waypoint_nav/waypoint_align_min_goal_distance` | `1.20 m` | `safety_node.launch` | Ít align với goal gần | Vẫn align khi goal gần |
| Hệ số P quay align | `/outdoor_waypoint_nav/waypoint_align_kp` | `1.6` | `safety_node.launch` | Quay mạnh/nhanh hơn | Quay mềm/chậm hơn |
| Tốc quay align tối đa | `/outdoor_waypoint_nav/waypoint_align_max_angular_speed` | `0.90 rad/s` | `safety_node.launch` | Quay nhanh hơn | Quay chậm/mượt hơn |

## 5. move_base

| Muốn chỉnh hành vi | Tham số | Giá trị hiện tại | File | Tăng lên thì sao | Giảm xuống thì sao |
|---|---:|---:|---|---|---|
| Tần số gửi lệnh vận tốc | `controller_frequency` | `10 Hz` | `move_base_gps_sim.launch` | Lệnh mượt hơn, nặng hơn | Lệnh thưa hơn |
| Tần số global planner | `planner_frequency` | `5 Hz` | `move_base_gps_sim.launch` | Replan nhanh hơn, có thể nhấp nháy | Replan chậm hơn |
| Controller patience | `controller_patience` | `5 s` | `move_base_gps_sim.launch` | Kiên nhẫn hơn khi kẹt | Báo lỗi nhanh hơn |
| Planner patience | `planner_patience` | `5 s` | `move_base_gps_sim.launch` | Chờ planner lâu hơn | Báo lỗi nhanh hơn |
| Oscillation timeout | `oscillation_timeout` | `5 s` | `move_base_gps_sim.launch` | Ít báo oscillation hơn | Báo kẹt nhanh hơn |
| Oscillation distance | `oscillation_distance` | `0.08 m` | `move_base_gps_sim.launch` | Cần tiến rõ hơn mới reset | Dễ reset hơn |
| Recovery mặc định | `recovery_behavior_enabled` | `false` | `move_base_gps_sim.launch` | Bật có thể tự clear costmap | Tắt tránh tự xoay/xóa map |

## 6. Joy/keyboard và RViz/debug

| Cần chỉnh/xem | Tham số hoặc topic | Giá trị/topic | File |
|---|---|---|---|
| Nút gửi waypoint | `/outdoor_waypoint_nav/send_button_num`, `send_button_sym` | `5`, `RB` | `src/outdoor_waypoint_nav/launch/simulation/joy_launch_control_sim.launch` |
| Nút abort/dừng | `/outdoor_waypoint_nav/abort_button_num`, `abort_button_sym` | `1`, `B` | `joy_launch_control_sim.launch` |
| Nút continue | `/outdoor_waypoint_nav/continue_button_num`, `continue_button_sym` | `3`, `Y` | `joy_launch_control_sim.launch` |
| Global path | `/move_base/NavfnROS/plan` | RViz `Global Plan` | `src/outdoor_waypoint_nav/rviz_config/model.rviz` |
| Local path | `/move_base/TrajectoryPlannerROS/local_plan` | RViz `Local Plan` | `model.rviz` |
| Lệnh trước safety | `/cmd_vel_intermediate` | MPC/local planner output trước lọc | `move_base_gps_sim.launch` |
| Lệnh sau safety | `/cmd_vel` | Lệnh robot thực nhận | `safety_node.launch` |
| Current goal | `/move_base/current_goal` | Safety dùng để giảm tốc/align | `safety_node.launch` |

## Tune nhanh theo triệu chứng

| Triệu chứng | Chỉnh trước | Hướng chỉnh |
|---|---|---|
| Xe vẫn chậm | `max_vel_x`, `acc_lim_x`, `goal_slowdown_distance`, `goal_slowdown_max_forward_speed` | So `/cmd_vel_intermediate` với `/cmd_vel`; nếu output chậm là safety đang kẹp |
| Xe lao quá waypoint | `goal_slowdown_distance`, `goal_slowdown_stop_distance`, `goal_slowdown_min_forward_speed`, `waypoint_advance_radius` | Tăng vùng giảm tốc hoặc tăng bán kính chuyển waypoint |
| Đổi waypoint bị lượn sóng | `waypoint_align_*`, `max_waypoint_goal_yaw_delta`, `waypoint_heading_preview_radius` | Bật align, giảm yaw delta, preview vừa đủ |
| Né vật cản sát | `occdist_scale`, `local_costmap/inflation_radius`, `cost_scaling_factor`, `safety_slowdown_distance` | Tăng obstacle weight/inflation, giảm `cost_scaling_factor` nếu muốn vùng cost lan rộng |
| Robot giật/dừng từng đoạn | `planner_frequency`, `controller_frequency`, `auto_replan_current_goal_interval`, `sim_time`, `vx_samples/vtheta_samples` | Giảm replan quá dày hoặc giảm số mẫu nếu CPU yếu |
| Local path quá ngắn | `sim_time`, `heading_lookahead`, `local_costmap/width/height` | Tăng `sim_time` trước, sau đó tăng vùng local nếu cần |
