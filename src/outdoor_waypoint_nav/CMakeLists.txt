cmake_minimum_required(VERSION 3.0.3)
project(outdoor_waypoint_nav)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  roslib
  roslaunch
  robot_localization
  visualization_msgs
)

catkin_package()
roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(gps_waypoint src/gps_waypoint.cpp)
#add_executable(gps_waypoint_continuous src/gps_waypoint_continuous.cpp)
add_executable(gps_waypoint_continuous1 src/gps_waypoint_continuous1.cpp)
add_executable(gps_waypoint_continuous2 src/gps_waypoint_continuous2.cpp)
add_executable(gps_waypoint_mapping src/gps_waypoint_mapping.cpp)
add_executable(collect_gps_waypoints src/collect_gps_waypoints.cpp)
add_executable(publish_waypoints_from_file src/publish_waypoints_from_file.cpp)
add_executable(plot_gps_waypoints src/plot_gps_waypoints.cpp)
add_executable(calibrate_heading src/calibrate_heading.cpp)
add_executable(safety_node src/safety_node.cpp)
add_executable(switch_controllers src/switch_controllers.cpp)

target_link_libraries(gps_waypoint ${catkin_LIBRARIES})
# target_link_libraries(gps_waypoint_continuous ${catkin_LIBRARIES})
target_link_libraries(gps_waypoint_continuous1 ${catkin_LIBRARIES})
target_link_libraries(gps_waypoint_continuous2 ${catkin_LIBRARIES})
target_link_libraries(gps_waypoint_mapping ${catkin_LIBRARIES})
target_link_libraries(collect_gps_waypoints ${catkin_LIBRARIES})
target_link_libraries(publish_waypoints_from_file ${catkin_LIBRARIES})
target_link_libraries(plot_gps_waypoints ${catkin_LIBRARIES})
target_link_libraries(calibrate_heading ${catkin_LIBRARIES})
target_link_libraries(safety_node ${catkin_LIBRARIES})
target_link_libraries(switch_controllers ${catkin_LIBRARIES})

install(DIRECTORY  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/camera_node.py
  scripts/position_error.py
  scripts/encoder_odometry_node.py
  scripts/experiment_trajectory_csv_logger.py
  scripts/filtered_collect_gps_waypoints.py
  scripts/gnss_correlated_noise.py
  scripts/gnss_outage_gate.py
  scripts/gnss_periodic_gate.py
  scripts/gps_kalman_filter.py
  scripts/gps_covariance_relay.py
  scripts/home_to_initial_waypoint.py
  scripts/imu_covariance_log.py
  scripts/imu_covariance_relay.py
  scripts/inovation.py
  scripts/inovation_sim.py
  scripts/joy_launch_control.py
  scripts/joy_launch_control_continuous.py
  scripts/joy_launch_control_mapping.py
  scripts/keyboard_to_joy.py
  scripts/path_node.py
  scripts/plot_results.py
  scripts/square_eval.py
  scripts/stationary_gps_gate.py
  scripts/tracking_control.py
  scripts/return_home.py
  scripts/record_gps_waypoints.py
  scripts/scenario_4_waypoint_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
