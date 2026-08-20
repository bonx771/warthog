cmake_minimum_required(VERSION 3.0.2)
project(imu_pkg)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  InsState15.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/imu_node.py
  scripts/send_data.py
  scripts/start_calib.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  launch/imu_ins.launch
  launch/send_data.launch
  launch/start_calib.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES
  config/imu_ins_template.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
