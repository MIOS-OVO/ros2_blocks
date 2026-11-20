cmake_minimum_required(VERSION 3.8)
project(recognized_blocks)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(block_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(detector src/detector.cpp)
target_compile_features(detector PRIVATE cxx_std_17)
target_include_directories(detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(detector ${OpenCV_LIBS})
ament_target_dependencies(detector
  ament_index_cpp block_interfaces geometry_msgs rclcpp sensor_msgs visualization_msgs)

add_executable(select_color src/select_color.cpp)
target_compile_features(select_color PRIVATE cxx_std_17)
ament_target_dependencies(select_color block_interfaces rclcpp)

add_executable(bringup src/bringup.cpp)
target_compile_features(bringup PRIVATE cxx_std_17)
ament_target_dependencies(bringup ament_index_cpp)

install(TARGETS detector select_color bringup DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY images DESTINATION share/${PROJECT_NAME})

ament_package()
