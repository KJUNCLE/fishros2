cmake_minimum_required(VERSION 3.8)
project(ros2_cpp)
add_executable(ros2_cpp_node ros2_cpp_node.cpp)

# 查找rclcpp头文件和库文件的路径
find_package(rclcpp REQUIRED)

# 添加rclcpp头文件路径
target_include_directories(ros2_cpp_node PRIVATE ${rclcpp_INCLUDE_DIRS})

# 链接rclcpp库文件
target_link_libraries(ros2_cpp_node ${rclcpp_LIBRARIES})