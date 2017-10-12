cmake_minimum_required(VERSION 2.8.3)
project(pg_driver)

add_compile_options(-std=c++11)
find_package(OpenCV)

find_package(catkin REQUIRED COMPONENTS
  rosconsole
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages                       ##
################################################
add_message_files(
   FILES
   Image.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/lib/libSpinnaker.so)


add_executable(singleCam_node src/singleCam_node.cpp)
add_dependencies(singleCam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(singleCam_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EXTRALIB_BIN})

add_executable(single_sub_node src/singleCam_sub_node.cpp)
add_dependencies(single_sub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(single_sub_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(multiCam_node src/multiCam_node.cpp)
add_dependencies(multiCam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multiCam_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EXTRALIB_BIN})

add_executable(multi_sub_node src/multiCam_sub_node.cpp)
add_dependencies(multi_sub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multi_sub_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
