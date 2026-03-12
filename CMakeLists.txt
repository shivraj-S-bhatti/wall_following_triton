cmake_minimum_required(VERSION 3.0.2)
project(wall_following_triton)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/pose_capture.py
  scripts/rl_d2_agent.py
  scripts/state_encoder.py
  scripts/triton_key_teleop.py
  scripts/wf_policy_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/clean_roslaunch.sh
  scripts/d2_state.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
