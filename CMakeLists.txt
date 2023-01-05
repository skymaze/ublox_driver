cmake_minimum_required(VERSION 3.5)
project(ublox_driver)
find_package(rclpy REQUIRED)
find_package(gnss_interfaces REQUIRED)
find_package(gnss_comm REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

set(INCLUDE_DIRS include ${rclpy_INCLUDE_DIRS} ${gnss_interfaces_INCLUDE_DIRS} ${gnss_comm_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS} ${nav_msgs_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS}
  ${ament_cmake_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/include/yaml)
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${rclpy_LIBRARY_DIRS} ${gnss_interfaces_LIBRARY_DIRS} ${gnss_comm_LIBRARY_DIRS}
  ${sensor_msgs_LIBRARY_DIRS} ${nav_msgs_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS}
  ${ament_cmake_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${rclpy_LIBRARIES} ${gnss_interfaces_LIBRARIES} ${gnss_comm_LIBRARIES} ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES} ${rclcpp_LIBRARIES} ${ament_cmake_LIBRARIES}
  ${std_msgs_LIBRARIES})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3 REQUIRED)
find_package(Glog REQUIRED)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/ublox_driver.cpp
  ${PROJECT_SOURCE_DIR}/src/ublox_message_processor.cpp
  ${PROJECT_SOURCE_DIR}/src/file_loader.cpp
  ${PROJECT_SOURCE_DIR}/src/file_dumper.cpp
  ${PROJECT_SOURCE_DIR}/src/serial_handler.cpp
  ${PROJECT_SOURCE_DIR}/src/socket_handler.cpp
  ${PROJECT_SOURCE_DIR}/third_party/yaml/Yaml.cpp)
target_link_libraries(${PROJECT_NAME} ${LIBS} ${GLOG_LIBRARIES})

add_executable(sync_system_time ${PROJECT_SOURCE_DIR}/src/sync_system_time.cpp)
target_link_libraries(sync_system_time ${LIBS})

ament_export_dependencies(rclpy)
ament_export_dependencies(gnss_interfaces)
ament_export_dependencies(gnss_comm)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(std_msgs)
ament_export_include_directories(${INCLUDE_DIRS})

install(TARGETS
  ${PROJECT_NAME}
  sync_system_time
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
