cmake_minimum_required(VERSION 3.8)
project(agv_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rviz2 REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)

# Install all resource folders into share/
install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY urdf    DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY assets  DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY rviz    DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY params  DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY worlds  DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY maps    DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY models  DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS
  scripts/fork_elevator_node.py
  scripts/teleop_forklift.py
  scripts/spawn_palets.py
  scripts/pallet_approach.py
  scripts/pallet_cycle_bt.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()