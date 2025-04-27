cmake_minimum_required(VERSION 3.8)
project(serial_package)

# Check for GCC or Clang and add compile options if using C++
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Ajouter l'exécutable
add_executable(UART_Node src/UART_Node.cpp)
add_executable(OdometryNode src/OdometryNode.cpp)


# Lier les bibliothèques nécessaires
ament_target_dependencies(UART_Node rclcpp std_msgs)
ament_target_dependencies(OdometryNode
  rclcpp
  std_msgs
  geometry_msgs
)

# Installer l'exécutable
install(TARGETS
  UART_Node
  OdometryNode
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS
    src/serial_node.py  # Make sure this matches your script's location
    src/serial_node_2.py
    src/serial_node_3.py
    src/serial_node_4.py
    src/serial_node_5.py
    src/serial_node_6.py
    src/test_uart.py
    src/test_uart_lidar.py	
    src/test_uart_lidar2.py
    src/robot_control_node.py
    src/robot_control_node_2.py
    src/robot_control_node_3.py
    src/robot_control_node_4.py
    src/robot_control_node_5.py
    src/robot_control_node_6.py
    src/robot_control_node_7.py
    src/screen_reader.py
    src/bagfile_stm32.py
    launch/launch_serial_lidar.py
    launch/launch_robot.py
    src/tirette_node.py
 

  DESTINATION lib/${PROJECT_NAME}  # Installs to lib/serial_package
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  # Skip the linter for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  
  # Skip cpplint checks
  set(ament_cmake_cpplint_FOUND TRUE)
  
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# Uncomment and add other dependencies as needed
# find_package(<dependency> REQUIRED)
