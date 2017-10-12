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


add_executable(${PROJECT_NAME}_node src/pg_driver_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EXTRALIB_BIN})

add_executable(mutiCam_node src/multiCam_node.cpp)
add_dependencies(mutiCam_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mutiCam_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${EXTRALIB_BIN})

add_executable(image_sub_node src/image_sub_node.cpp)
add_dependencies(image_sub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_sub_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(multiImage_sub_node src/multiImage_sub_node.cpp)
add_dependencies(multiImage_sub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(multiImage_sub_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
