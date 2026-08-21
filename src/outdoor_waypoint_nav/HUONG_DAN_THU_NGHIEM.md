# Hướng dẫn chạy thử nghiệm Kịch bản 1, 2, 3 và 4

Tài liệu này áp dụng cho package `outdoor_waypoint_nav` trên Warthog thật.
Mục tiêu là chạy cùng một tuyến khoảng 300 m, 5 waypoint và cùng vận tốc để
so sánh kết quả giữa GNSS bình thường (Kịch bản 1), GNSS bị suy giảm có kiểm
soát (Kịch bản 2) và mất GNSS tạm thời (Kịch bản 3).

> Hướng dẫn riêng cho Gazebo/mô phỏng Kịch bản 1–4 nằm tại
> `HUONG_DAN_MO_PHONG_KICH_BAN_1_4.md`.

> **Cảnh báo cấu hình hiện tại:** `waypoint_files/points_outdoor.txt` đang có
> 3 waypoint, trong khi mô tả thí nghiệm và Kịch bản 2 (WP2 → WP4) cần tối
> thiểu 4 waypoint, mục tiêu là 5 waypoint. Logger CSV tự đọc số waypoint thật
> của file, nhưng điều đó không làm profile WP2 → WP4 trở nên hợp lệ. Hãy chọn
> hoặc khôi phục file tuyến 5 waypoint trước khi chạy Kịch bản 2; file hiện tại
> chưa bị tự ý thay đổi.

## 1. Quy ước, an toàn và điều kiện chung

Các launch K1–K4 hiện tắt bán kính chuyển waypoint sớm
(`waypoint_advance_radius=0`). Direct-pursuit chỉ xác nhận waypoint khi pose
ước lượng cách tâm waypoint không quá 0,05 m; đây là giới hạn nhỏ nhất đã được
khóa trong C++ và là giá trị gần 0 nhất có thể dùng mà không sửa code. Với
GNSS thực, ngưỡng 5 cm rất chặt và có thể làm xe chờ hoặc timeout nếu độ nhiễu
định vị lớn hơn ngưỡng này.

- Luôn có người giữ điều khiển từ xa, khu vực chạy phải không có người/vật cản
  trên tuyến. Nút dừng khẩn phần cứng vẫn là phương án dừng ưu tiên.
- Đặt xe đúng cùng vị trí và hướng xuất phát cho mọi lần chạy. Không thay đổi
  file waypoint, vận tốc, cấu hình EKF, IMU hoặc encoder giữa các lần trong
  cùng một nhóm so sánh.
- Chỉ thu waypoint ở chế độ GPS sạch. Không thu lại waypoint trong lúc Kịch
  bản 2 đang bật nhiễu hoặc Kịch bản 3 đang cắt GNSS.
- Đợi GNSS có fix tốt và các topic EKF ổn định trước khi bắt đầu. Khi thay đổi
  hướng xuất phát, thực hiện heading calibration lại.
- Các lệnh dưới đây giả sử workspace là `/home/bo/warthog`. Nếu terminal
  của bạn chưa source workspace, chạy:

  ```bash
  source /opt/ros/noetic/setup.bash
  source /home/bo/warthog/devel/setup.bash
  ```

  Nếu chưa có thư mục `devel`, cần build workspace một lần bằng `catkin_make`
  trong `/home/bo/warthog`, sau đó source lại `devel/setup.bash`.

### Quy tắc chạy hai terminal

Kịch bản 1–3 đều dùng cùng Terminal 1 để chạy stack GPS/EKF/navigation.
Terminal 2 chạy **một** launch kịch bản tương ứng. Launch K1–K3 đã include
nguyên `joy_launch_control.launch`, node đánh giá sai số thực tế và logger
CSV, vì vậy không mở riêng Terminal 3 và không chạy
`joy_launch_control.launch` riêng. K4 có bộ điều khiển riêng để tránh xung
đột phím; xem phần K4 bên dưới.

`navsat_transform` trong Terminal 1 luôn nhận GPS từ topic trung gian
`/outdoor_waypoint_nav/gps/fix_selected`. Launch Kịch bản ở Terminal 2 là
publisher duy nhất của topic này: GPS sạch cho Kịch bản 1, GPS nhiễu cho Kịch
bản 2, hoặc GPS bị cắt/khôi phục cho Kịch bản 3. Khởi động Terminal 1 trước,
sau đó khởi động Terminal 2 và đợi GNSS/EKF ổn định rồi mới bấm RB/`r`.

Trong cả K1-K4, bấm `h` (Home) trên bàn phím để dừng waypoint process đang
chạy, hủy goal `move_base` còn treo và lái UGV trực tiếp về waypoint ban đầu
của route đang active. Với K1-K3, waypoint ban đầu là dòng đầu của file
`coordinates_file`; với K4, lệnh này chỉ hợp lệ sau khi `y` đã sinh và khóa
route. Xem `h` như kết thúc/huỷ run hiện tại: sau khi xe về WP1, kiểm tra vùng
an toàn rồi dừng và khởi động lại Terminal 2 trước lần đo tiếp theo. Report
`position_error.png` của lần home sẽ tính sai số khoảng cách tới WP1; run hoàn
thành bằng `r` như bình thường vẫn tính sai số tới waypoint cuối.

Sau mỗi lần chạy K1–K3, dừng rồi khởi động lại launch ở Terminal 2 trước lần
tiếp theo; Terminal 1 có thể giữ nguyên. Cách này tạo report/CSV mới cho mỗi
run, tạo chuỗi nhiễu mới ở Kịch bản 2 và re-arm node GNSS OFF ở Kịch bản 3.

### CSV quỹ đạo và Cross-track RMSE

Sau khi bấm RB/`r`, logger bắt đầu khi `gps_waypoint` phát
`waypoint_reached_index=0`; vì thế dữ liệu chuẩn bị/đặt xe trước khi gửi
waypoint không lẫn vào run. Khi chạy xong hoặc dừng Terminal 2, node đóng các
file tại:

```text
/home/bo/warthog/src/outdoor_waypoint_nav/results/experiment_runs/
  <scenario>_<YYYYMMDD_HHMMSS>/
```

- `waypoints.csv`: tuyến lý tưởng, gồm latitude/longitude, local ENU (gốc là
  WP1), UTM và map khi TF `utm -> map` đã sẵn sàng.
- `reference_gnss.csv`: quỹ đạo reference từ `/gps/fix` sạch, cùng các cột
  CTE theo từng đoạn waypoint. Đây là file dùng để tính RMSE.
- `controller_trajectory.csv`: pose `filtered_map` và target mà controller
  direct-pursuit đang dùng; đây là quỹ đạo ước lượng của controller, không
  phải ground truth độc lập.
- `position_error.png`: hình quỹ đạo EKF và
  sai số vị trí cuối; được lưu ngay trong cùng thư mục run với các CSV trên.

Logger vẫn tính Cross-track RMSE khi kết thúc, in giá trị ở Terminal 2 và phát
trên topic `/outdoor_waypoint_nav/experiment_logger/cross_track_rmse_m`.

Để vẽ quỹ đạo thực tế so với tuyến lý tưởng, dùng cột `local_east_m`,
`local_north_m` trong `reference_gnss.csv` cùng `waypoints.csv`. Nếu muốn vẽ
chồng quỹ đạo EKF/direct-pursuit, dùng các cột `map_x_m`, `map_y_m` của
`controller_trajectory.csv`, `reference_gnss.csv` và `waypoints.csv` (các cột
map chỉ có khi TF `utm -> map` sẵn sàng).

`cross_track_rmse_m` được tính theo `sqrt(mean(CTE_i^2))` trong hệ UTM mét
(dự phòng local ENU nếu thiếu bộ chuyển đổi), trong đó CTE là khoảng cách từ
từng mẫu GNSS sạch đến **đoạn hữu hạn đang chạy** WP1→WP2, WP2→WP3, ...; đoạn
từ vị trí xuất phát đến WP1 không được đưa vào RMSE. Gán đoạn bằng waypoint
gần nhất đã đạt, không lấy đoạn gần nhất tùy ý, nên sẽ không che mất trường
hợp xe đi sai tuyến. `/gps/fix` ở đây là GNSS reference; chỉ gọi là ground
truth khi nguồn này được kiểm chứng độc lập, ví dụ RTK Fixed/bộ đo chuẩn.

## 2. Kịch bản 1 — Điều kiện GNSS bình thường

### Mục tiêu

- Tuyến: khoảng 300 m, 5 waypoint.
- Chạy 5 lần liên tiếp.
- Đạt khi cả 5 lần có sai số tại đích thực nhỏ hơn 3 m.

### Cách chạy một lần

Terminal 1, khởi động hệ dẫn đường GPS/EKF bình thường:

```bash
roslaunch outdoor_waypoint_nav outdoor_waypoint_nav.launch
```

Terminal 2, khởi động Kịch bản 1:

```bash
roslaunch outdoor_waypoint_nav scenario_1_gnss_normal.launch
```

Launch này relay GPS sạch, đồng thời include toàn bộ joystick/keyboard control
của `joy_launch_control.launch`, node đánh giá và logger CSV. Khi waypoint
hoàn tất, ảnh kết quả tự lưu trong `outdoor_waypoint_nav/results/` và CSV của
run tự lưu trong `outdoor_waypoint_nav/results/experiment_runs/`.

Sau khi kiểm tra RViz, GNSS và EKF ổn định:

1. Đưa xe về đúng mốc đầu, tránh chạy vòng thêm trước khi bắt đầu.
2. Bấm RB/`r` để gửi 5 waypoint.
3. Theo dõi xe và sẵn sàng dừng khẩn nếu có nguy cơ va chạm.
4. Khi hoàn tất, ghi sai số EKF, sai số đo thực tế, thời gian, thời tiết, trạng
   thái RTK/fix và mọi sự cố vào bảng kết quả.
5. Dừng các node/bag, đưa xe lại đúng mốc đầu rồi thực hiện lần tiếp theo.

Mẫu bảng kết quả:

| Lần | Sai số EKF cuối (m) | Sai số đo thực (m) | Cross-track RMSE theo GPS sạch (m) | Thời gian (s) | Trạng thái GNSS | Đạt `<3 m`? | Ghi chú |
| --- | ---: | ---: | ---: | ---: | --- | --- | --- |
| 1 |  |  |  |  |  |  |  |
| 2 |  |  |  |  |  |  |  |
| 3 |  |  |  |  |  |  |  |
| 4 |  |  |  |  |  |  |  |
| 5 |  |  |  |  |  |  |  |

## 3. Kịch bản 2 — GNSS liên tục nhưng có nhiễu tương quan

### Luồng và nguyên tắc

Launch Kịch bản 2 sử dụng luồng sau:

```text
/gps/fix -> gnss_correlated_noise -> /gps/fix_noisy
                                      -> relay
                                      -> /outdoor_waypoint_nav/gps/fix_selected
                                      -> navsat_transform -> EKF
```

`/gps/fix` sạch vẫn được giữ để log/đối chiếu. Node nhiễu chỉ làm lệch hai trục
ngang East/North; nó giữ nguyên timestamp, trạng thái fix và altitude. Node
cộng variance nhiễu vào covariance gốc của F9P, nên EKF không bị buộc phải tin
quá mức vào dữ liệu đã bị làm xấu. 

Mô hình đúng theo

```text
n_k = alpha * n_(k-1) + (1 - alpha) * w_k
w_k ~ N(0, innovation_sigma_m^2), alpha = 0.9
```

Với `alpha=0.9`, độ lệch chuẩn ổn định trên mỗi trục là
`innovation_sigma_m / sqrt(19)`:

| Mức | `innovation_sigma_m` | Std đầu ra mỗi trục | Variance cộng thêm mỗi trục |
| --- | ---: | ---: | ---: |
| Nhẹ | 3 m | 0.69 m | 0.474 m² |
| Trung bình | 6 m | 1.38 m | 1.895 m² |
| Lớn | 9 m | 2.06 m | 4.263 m² |

### Vùng nhiễu chính thức: từ WP2 đến WP4

Kịch bản 2 mặc định dùng GPS sạch cho đoạn xuất phát → WP2. Ngay sau khi
controller xác nhận **đã đạt WP2**, node bắt đầu thêm nhiễu. Nhiễu tồn tại trên
hai đoạn WP2 → WP3 và WP3 → WP4. Ngay khi controller xác nhận **đã đạt WP4**,
node lập tức trả về `/gps/fix` sạch; đoạn WP4 → đích (và hành trình quay về nếu
bật) dùng GPS tốt.

`gps_waypoint` publish topic trạng thái
`/outdoor_waypoint_nav/waypoint_reached_index`; node nhiễu dùng topic này thay
vì ước lượng theo mét, nên mốc bật/tắt bám đúng waypoint. Sau khi cập nhật C++
này, build lại workspace trước khi chạy:

```bash
cd /home/bo/warthog
catkin_make
source devel/setup.bash
```

Terminal 1 (giống mọi kịch bản):

```bash
roslaunch outdoor_waypoint_nav outdoor_waypoint_nav.launch
```

Terminal 2: chạy một trong các mức Kịch bản 2 sau:

```bash
# Nhẹ, lần 1
roslaunch outdoor_waypoint_nav scenario_2_gnss_noise.launch \
  innovation_sigma_m:=3.0

# Trung bình, lần 1
roslaunch outdoor_waypoint_nav scenario_2_gnss_noise.launch \
  innovation_sigma_m:=6.0

# Lớn, lần 1
roslaunch outdoor_waypoint_nav scenario_2_gnss_noise.launch \
  innovation_sigma_m:=9.0
```

Launch Kịch bản 2 đã chứa joystick/keyboard control, node đánh giá và logger
CSV; không chạy thêm `joy_launch_control.launch`, evaluator hoặc logger riêng.
Sau WP2, log phải có `GNSS noise enabled after reached WP2`; sau WP4 phải có
`GNSS noise disabled at reached WP4`. Có thể kiểm tra nhanh:

```bash
rostopic echo /outdoor_waypoint_nav/waypoint_reached_index
rostopic echo /outdoor_waypoint_nav/gnss_noise/active
rostopic echo /outdoor_waypoint_nav/gnss_noise/offset_enu
```

#### Lần 1 và Lần 2

Mỗi lần khởi động `scenario_2_gnss_noise.launch`, node tự tạo một chuỗi nhiễu
mới. Vì vậy chỉ cần chạy cùng mức nhiễu hai lần độc lập, ghi kết quả là Lần 1
và Lần 2.

### Bảng kết quả Kịch bản 2

| Mức | Lần | Profile | Sai số EKF (m) | Sai số đo thực (m) | Cross-track RMSE theo GPS sạch (m) | Max \|E/N noise\| (m) | Đạt? | Ghi chú |
| --- | ---: | --- | ---: | ---: | ---: | ---: | --- | --- |
| Nhẹ | 1 | WP2 → WP4 |  |  |  |  |  |  |
| Nhẹ | 2 | WP2 → WP4 |  |  |  |  |  |  |
| Trung bình | 1 | WP2 → WP4 |  |  |  |  |  |  |
| Trung bình | 2 | WP2 → WP4 |  |  |  |  |  |  |
| Lớn | 1 | WP2 → WP4 |  |  |  |  |  |  |
| Lớn | 2 | WP2 → WP4 |  |  |  |  |  |  |

## 4. Kịch bản 3 — Mất GNSS tạm thời sau WP2

### Mục tiêu và nguyên tắc

Kịch bản này đánh giá khả năng duy trì dẫn đường bằng wheel odometry và IMU
khi GNSS mất hoàn toàn trong một đoạn đường. Xe vẫn chạy cùng tuyến và cùng
5 waypoint như Kịch bản 1; chỉ thay đổi trạng thái GNSS như sau:

- Từ điểm xuất phát đến khi đạt WP2: GPS sạch.
- Ngay sau khi controller xác nhận đạt WP2: ngắt GNSS.
- Sau khi xe đi thêm lần lượt 20 m, 40 m hoặc 60 m: bật GNSS lại.
- Từ thời điểm GNSS bật lại đến đích: GPS sạch.

Luồng dữ liệu của launch Kịch bản 3 là:

```text
/gps/fix -> gnss_outage_gate -> /gps/fix_off
                                 -> relay
                                 -> /outdoor_waypoint_nav/gps/fix_selected
                                 -> navsat_transform -> EKF
```

Trong thời gian OFF, `gnss_outage_gate` **không publish bất kỳ** bản tin
`NavSatFix` nào sang `/gps/fix_off`; đây là mô phỏng mất receiver GNSS, không
phải là GPS có tọa độ giả. Quãng đường 20/40/60 m được tích lũy theo chiều dài
đường đi từ `/outdoor_waypoint_nav/odometry/filtered_odom`. Đây là EKF1 chỉ
dùng encoder và IMU, không dùng GPS, nên phù hợp để đo đoạn dead-reckoning.

### Chạy từng mức

Terminal 1 (giống mọi kịch bản):

```bash
roslaunch outdoor_waypoint_nav outdoor_waypoint_nav.launch
```

Terminal 2: chạy một trong các mức Kịch bản 3 sau. Launch này đã chứa node
cắt GNSS, joystick/keyboard control, node đánh giá và logger CSV.

```bash
# Mất GNSS trong 20 m, Lần 1
roslaunch outdoor_waypoint_nav scenario_3_gnss_outage.launch \
  outage_distance_m:=20
 
# Mất GNSS trong 40 m, Lần 1
roslaunch outdoor_waypoint_nav scenario_3_gnss_outage.launch \
  outage_distance_m:=40

# Mất GNSS trong 60 m, Lần 1
roslaunch outdoor_waypoint_nav scenario_3_gnss_outage.launch \
  outage_distance_m:=60
```

Khi GNSS/EKF ổn định, đặt xe tại mốc đầu và bấm RB/`r` để gửi 5 waypoint. Có
thể quan sát trực tiếp trạng thái Kịch bản 3 bằng:

```bash
rostopic echo /outdoor_waypoint_nav/waypoint_reached_index
rostopic echo /outdoor_waypoint_nav/gnss_outage/active
rostopic echo /outdoor_waypoint_nav/gnss_outage/distance_m
```

`active: True` nghĩa là GNSS đang bị cắt. `distance_m` tăng từ 0 sau WP2;
khi đạt đúng giá trị `outage_distance_m`, node chuyển `active: False` và tiếp
tục relay GPS sạch. Kiểm tra thêm `/gps/fix_off`: topic này phải ngừng có bản
tin trong lúc OFF và có lại khi GNSS được bật.

#### Lần 1 và Lần 2

Với mỗi mức 20/40/60 m, chạy hai lần độc lập. Sau khi kết thúc một lần, dừng
`scenario_3_gnss_outage.launch`, đưa xe về mốc đầu, rồi khởi động lại launch
cho lần kế tiếp. Node cắt GNSS là one-shot, nên restart launch là cần thiết để
nó lại bắt đầu cắt ngay sau WP2.

Khi GPS quay lại, EKF có thể cần một khoảng ngắn để tái hội tụ; nếu dead
reckoning trôi quá lớn, correction GPS cũng có thể bị EKF từ chối theo ngưỡng
cấu hình hiện tại. Giữ nguyên cấu hình EKF giữa các lần chạy và ghi rõ hiện
tượng này vào kết quả, thay vì đổi ngưỡng để làm đẹp số liệu.

### Bảng kết quả Kịch bản 3

| Mức | Lần | Profile | Sai số EKF cuối (m) | Sai số đo thực (m) | Cross-track RMSE theo GPS sạch (m) | Sai số cực đại khi OFF (m) | GPS hồi phục? | Ghi chú |
| --- | ---: | --- | ---: | ---: | ---: | ---: | --- | --- |
| 20 m | 1 | WP2 → WP2 + 20 m |  |  |  |  |  |  |
| 20 m | 2 | WP2 → WP2 + 20 m |  |  |  |  |  |  |
| 40 m | 1 | WP2 → WP2 + 40 m |  |  |  |  |  |  |
| 40 m | 2 | WP2 → WP2 + 40 m |  |  |  |  |  |  |
| 60 m | 1 | WP2 → WP2 + 60 m |  |  |  |  |  |  |
| 60 m | 2 | WP2 → WP2 + 60 m |  |  |  |  |  |  |

## 5. Kịch bản 4 — Ảnh hưởng của hình dạng quỹ đạo đến định vị

Kịch bản 4 dùng **một** waypoint GPS thực tế (WP1) và hướng hiện tại của
Warthog để tính ra bốn waypoint còn lại. Vì vậy không cần đi thu WP2–WP5,
nhưng xe phải được đặt đúng tại WP1 và quay đúng theo trục giữa trước khi
nhấn 'y'.

Route đúng theo hình trong file PDF có 5 waypoint và bốn đoạn dài
'a, a, 2a, a', với:

~~~text
a = tổng_quãng_đường / 5
~~~

Với tổng 300 m, 'a = 60 m', nên route luôn có các đoạn '60, 60, 120, 60 m'.
Chỉ góc lệch 'theta_deg' thay đổi:

| Trường hợp | 'theta_deg' | Góc đổi hướng lớn nhất tại đỉnh |
| --- | ---: | ---: |
| TH1 | 15° | 30° |
| TH2 | 30° | 60° |
| TH3 | 45° | 90° |

Góc trong PDF là góc lệch từng đoạn xiên so với trục giữa. Do đó TH3 có hai
đỉnh đổi hướng 90°; direct-pursuit có thể quay gần tại chỗ ở các đỉnh đó. Giữ
nguyên tốc độ và toàn bộ tham số direct-pursuit giữa ba TH.

Mặc định route lệch phải ở WP2→WP3 rồi trở lại qua trái ở WP3→WP4. Nếu bãi
chạy chỉ trống về phía còn lại, thêm `first_side:=left` vào lệnh Terminal 2 để
lấy route đối xứng; không đổi giá trị này giữa các lần trong cùng một TH.

### Chạy một trường hợp

Terminal 1:

~~~bash
roslaunch outdoor_waypoint_nav outdoor_waypoint_nav.launch
~~~

Terminal 2: chọn một góc và file riêng để không ghi đè kết quả hình học của
các trường hợp khác:

~~~bash
# TH1
roslaunch outdoor_waypoint_nav scenario_4_trajectory.launch \
  theta_deg:=15 \
  waypoint_file:=/waypoint_files/points_scenario_4_th15.txt

# TH2
roslaunch outdoor_waypoint_nav scenario_4_trajectory.launch \
  theta_deg:=30 \
  waypoint_file:=/waypoint_files/points_scenario_4_th30.txt

# TH3
roslaunch outdoor_waypoint_nav scenario_4_trajectory.launch \
  theta_deg:=45 \
  waypoint_file:=/waypoint_files/points_scenario_4_th45.txt
~~~

Terminal 2 relay GPS sạch vào topic chọn cho EKF, chứa điều khiển phím K4,
evaluator và CSV logger; không chạy riêng 'joy_launch_control.launch'.
K4 dùng bộ điều khiển riêng vì 'l'/'r' của bộ điều khiển cũ sẽ mở collector và
sender của K1–K3, gây ghi đè file waypoint thường.

### Trình tự phím K4

1. 'l': reset phiên chuẩn bị K4 trong bộ nhớ và chờ nhận WP1. File route cũ
   chưa bị xóa.
2. Đưa xe đứng yên tại WP1, quay mũi xe theo trục giữa của bãi chạy, bảo đảm
   phía trước đủ hành lang trống.
3. 'y': giữ xe đứng yên trong 5 giây. Node lấy median '/gps/fix' làm WP1 và
   lấy heading từ TF 'utm -> base_link'. Nếu GPS/TF chưa ổn định, node từ chối
   mẫu và yêu cầu nhấn 'y' lại.
4. Sau 5 giây lấy mẫu, node tự tính WP2–WP5 trong UTM, đổi ngược về lat/lon,
   ghi file waypoint và file metadata JSON, validate/khóa route, đồng thời
   publish preview lên:

   ~~~text
   /outdoor_waypoint_nav/scenario_4/generated_waypoints
   ~~~

5. 'r': chỉ có hiệu lực sau khi bước 'y' đã sinh và khóa route; node mới khởi
   'gps_waypoint' để xe chạy.
6. 'h': sau khi route đã khóa, dừng route hiện tại nếu đang chạy và đưa xe về
   WP1 của chính route K4 đó. Nếu bấm trước 'y', node sẽ từ chối để tránh dùng
   nhầm file route cũ.

'b' gửi vận tốc 0 và hủy run đang chạy; 'h' là lệnh home có điều khiển về WP1.
Nút dừng khẩn phần cứng vẫn là lựa
chọn ưu tiên. Nếu cần heading calibration, thực hiện nó như bước chuẩn bị
riêng trước khi khởi Terminal 1/K4; không dùng phím calibration trong K4 vì
nó có thể lái xe tiến/lùi.

Route được tính trong UTM, không cộng trực tiếp vào latitude/longitude. File
metadata cạnh file waypoint lưu WP1, bearing, UTM EPSG, góc, từng waypoint và
độ dài segment để có thể kiểm chứng/sử dụng lại sau này. Sau mỗi run, dừng và
khởi động lại Terminal 2 trước lần đo tiếp theo. Nếu cần giữ nguyên route để
đối chiếu, sao chép file waypoint và metadata đã khóa trước khi lấy WP1 mới.

Với tổng 300 m, hành lang hình học tối thiểu (chưa gồm biên an toàn) là
khoảng 31.1 m, 60.0 m và 84.9 m tương ứng TH1, TH2, TH3. Chỉ chạy ở bãi
trống; thêm ít nhất 5–10 m biên an toàn mỗi bên, đặc biệt ở TH3.

K4 vẫn ghi CSV và ảnh đánh giá như các kịch bản trước. Khác với K1–K3, manager
phát event bắt đầu ngay sau khi khởi thành công 'gps_waypoint' ở phím 'r'; vì
thế đoạn thao tác 'l/y' không lẫn vào kết quả, đồng thời không bị mất event
nếu WP1 được đạt quá nhanh. Khi hoàn thành, node chốt CSV/PNG rồi khóa run;
dừng và khởi động lại Terminal 2 trước lần đo tiếp theo. Điều này tránh trộn
dữ liệu lúc report của lần trước còn đang được ghi. Nếu bấm 'b', sender lỗi,
hoặc không đạt đủ waypoint cũng phải khởi động lại Terminal 2.

Mẫu bảng kết quả:

| TH | Lần | Góc lệch (°) | Sai số EKF cuối (m) | Sai số đo thực (m) | Cross-track RMSE theo GPS sạch (m) | Hoàn thành đủ WP? | Ghi chú |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| TH1 | 1 | 15 |  |  |  |  |  |
| TH1 | 2 | 15 |  |  |  |  |  |
| TH2 | 1 | 30 |  |  |  |  |  |
| TH2 | 2 | 30 |  |  |  |  |  |
| TH3 | 1 | 45 |  |  |  |  |  |
| TH3 | 2 | 45 |  |  |  |  |  |

## 6. Điều kiện dừng và kiểm tra sau chạy

Dừng bài ngay khi xe mất kiểm soát, GNSS/EKF bất thường, odometry/TF thiếu,
hoặc có vật cản vào vùng an toàn. Không dùng các lần bị dừng khẩn để khẳng
định đạt; vẫn lưu bag và ghi rõ nguyên nhân.

Sau mỗi run, kiểm tra tối thiểu:

```bash
rostopic hz /gps/fix
rostopic hz /gps/fix_noisy                  # chỉ Kịch bản 2
rostopic hz /gps/fix_off                    # chỉ Kịch bản 3, sẽ im khi OFF
rostopic hz /outdoor_waypoint_nav/gps/fix_selected
rostopic echo -n 1 /outdoor_waypoint_nav/odometry/filtered_map
```

Khi phân tích Kịch bản 2, đối chiếu `/gps/fix`, `/gps/fix_noisy`,
`offset_enu`, pose EKF và sai số đích. Báo cáo nên nêu rõ mức nhiễu, `alpha`,
profile, điều kiện GNSS/RTK và cách đo ground truth.

Khi phân tích Kịch bản 3, đối chiếu thời điểm đạt WP2, thời điểm
`gnss_outage/active` chuyển ON/OFF, `distance_m`, pose EKF trong lúc OFF và
sai số sau khi GPS hồi phục.
