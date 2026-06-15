cmake_minimum_required(VERSION 3.14)
project(rosserial_server_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosserial_server REQUIRED)

add_executable(test_serial_node test/test_serial_node.cpp)
target_link_libraries(test_serial_node PRIVATE rosserial_server::rosserial_server)

install(TARGETS test_serial_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()