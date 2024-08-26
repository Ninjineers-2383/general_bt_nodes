cmake_minimum_required(VERSION 3.8)
project(general_bt_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(drive_action_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(drive_plugin SHARED src/drive_for_seconds.cpp )
target_compile_definitions(drive_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(drive_plugin behaviortree_ros2 drive_action_interfaces )

add_library(log_message SHARED src/log.cpp)
target_compile_definitions(log_message PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(log_message behaviortree_ros2 )

add_library(parallel_groups SHARED src/parallel_groups/parallel_groups.cpp)
target_include_directories(parallel_groups PRIVATE src/parallel_groups)
target_compile_definitions(parallel_groups PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(parallel_groups behaviortree_ros2 )

add_library(joint_trajectory SHARED src/joint_trajectory/joint_trajectory_nodes.cpp)
target_include_directories(joint_trajectory PRIVATE src/joint_trajectory)
target_compile_definitions(joint_trajectory PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(joint_trajectory behaviortree_ros2 control_msgs trajectory_msgs )

install(TARGETS
  drive_plugin
  log_message
  parallel_groups
  joint_trajectory
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
