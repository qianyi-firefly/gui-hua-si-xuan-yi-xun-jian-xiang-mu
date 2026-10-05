cmake_minimum_required(VERSION 3.0.2)
project(drone_stack)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs message_generation mavros_msgs nav_msgs roscpp rviz
  sensor_msgs std_srvs visualization_msgs
)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
add_service_files(FILES Takeoff.srv LocalGoal.srv SetNavigationSpeed.srv)
generate_messages(DEPENDENCIES geometry_msgs)
catkin_package(CATKIN_DEPENDS geometry_msgs mavros_msgs nav_msgs roscpp rviz sensor_msgs std_srvs visualization_msgs message_runtime)
include_directories(${catkin_INCLUDE_DIRS})
add_executable(drone_operator_gui src/operator_gui.cpp)
add_dependencies(drone_operator_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drone_operator_gui Qt5::Core Qt5::Widgets ${catkin_LIBRARIES})
catkin_install_python(PROGRAMS scripts/lio_bridge.py scripts/flight_manager.py scripts/sim_livox_adapter.py
  scripts/sim_camera_processor.py scripts/trajectory_guard.py scripts/safe_global_path.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS drone_operator_gui RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config worlds models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
