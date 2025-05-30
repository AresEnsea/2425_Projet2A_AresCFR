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
find_package(ament_index_cpp REQUIRED)

# Ajouter les exécutables C++
add_executable(UART_Node src/UART_Node.cpp)
add_executable(OdometryNode src/OdometryNode.cpp)
add_executable(InstructionNode src/instruction_node.cpp)

# Lier les bibliothèques nécessaires
ament_target_dependencies(UART_Node rclcpp std_msgs)
ament_target_dependencies(OdometryNode
  rclcpp
  std_msgs
  geometry_msgs
)
ament_target_dependencies(InstructionNode rclcpp std_msgs ament_index_cpp)

# Installer les exécutables
install(TARGETS
  UART_Node
  OdometryNode
  InstructionNode
  DESTINATION lib/${PROJECT_NAME}
)

# Installer les scripts Python classiques (sans renommage)
install(PROGRAMS
  src/screen_reader.py
  src/bag_player_node.py
  src/bag_listener.py
  src/serial_node.py
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
  src/keyboard_control.py
  src/bagfile_stm32.py
  launch/launch_serial_lidar.py
  launch/launch_robot.py
  src/keyboard_test.py
  src/keyboard_test_2.py
  src/keyboard_lidar_free.py
  src/bag_listener_2.py
  src/control_keyboard.py
  src/bag_playback.py
  DESTINATION lib/${PROJECT_NAME}
)

# Installer le script tirette.py sous le nom tirette (sans extension)
install(PROGRAMS
  src/tirette.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tirette
)

install(PROGRAMS
  src/bag_player_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bag_player_node
)

install(PROGRAMS
  src/bag_listener.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bag_listener
)

# Installer les fichiers de données
install(FILES
  instructions_jaune.txt
  instructions_bleu.txt
  instructions_test.txt
  DESTINATION share/${PROJECT_NAME}
)

# Installer les dossiers launch et bags
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY bags
  DESTINATION share/${PROJECT_NAME}
)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# Uncomment and add other dependencies as needed
# find_package(<dependency> REQUIRED)
