cmake_minimum_required(VERSION 3.0.2)
project(imu_bias_compensator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/gyro_calibrator.py
  scripts/zupt_compensator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
