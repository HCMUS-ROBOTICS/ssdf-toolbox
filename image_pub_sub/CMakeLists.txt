cmake_minimum_required(VERSION 3.0.2)
project(image_pub_sub)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  image_transport
  roscpp
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
 CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(image_pub_node src/image_pub_node.cpp)

target_link_libraries(
  image_pub_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(image_pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS image_pub_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
