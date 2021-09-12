cmake_minimum_required(VERSION 3.0.2)
project(image_pub_sub)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package(
 CATKIN_DEPENDS rospy sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS
  bin/image_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
