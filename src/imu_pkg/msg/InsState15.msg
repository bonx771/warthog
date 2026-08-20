# Header ROS chuẩn để đóng dấu thời gian và frame cho bản tin 15 state.
std_msgs/Header header

# 15 state bên dưới giữ đúng thứ tự state vector của robot_localization.
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
float64 vx
float64 vy
float64 vz
float64 vroll
float64 vpitch
float64 vyaw
float64 ax
float64 ay
float64 az

# Mỗi bit trong valid_mask cho biết state tương ứng có được giải mã hay không.
# Bit 0 -> x, bit 1 -> y, ..., bit 14 -> az.
uint32 valid_mask
