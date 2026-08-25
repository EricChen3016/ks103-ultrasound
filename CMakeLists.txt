cmake_minimum_required(VERSION 3.8)
project(ks103_ultrasound)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(ks103_driver src/ks103_bus.cpp src/ks103_node.cpp)
target_include_directories(ks103_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(ks103_driver rclcpp sensor_msgs)

add_executable(ks103_node src/main.cpp)
target_link_libraries(ks103_node ks103_driver)
ament_target_dependencies(ks103_node rclcpp)

install(TARGETS ks103_driver ks103_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ks103 test/test_ks103.cpp)
  target_link_libraries(test_ks103 ks103_driver)
  ament_target_dependencies(test_ks103 rclcpp sensor_msgs)
endif()

ament_export_include_directories(include)
ament_export_dependencies(rclcpp sensor_msgs)
ament_package()
