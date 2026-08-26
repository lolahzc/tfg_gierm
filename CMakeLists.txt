cmake_minimum_required(VERSION 3.5)
project(mission_planner)

# C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# Dependencies
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(as2_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(behaviortree_cpp_v3 QUIET)
find_package(yaml-cpp QUIET)
find_package(as2_core REQUIRED)

# -----------------------------------------------------------------------------
# Message and Action definitions
# -----------------------------------------------------------------------------
set(MSG_FILES
  "msg/AgentBeacon.msg"
  "msg/PlannerBeacon.msg"
  "msg/Waypoint.msg"
  "msg/MonitorParams.msg"
  "msg/MonitorUGVParams.msg"
  "msg/InspectParams.msg"
  "msg/DeliverToolParams.msg"
  "msg/DoCloserInspectionGoal.msg"
  "msg/RechargeParams.msg"
  "msg/WaitParams.msg"
  "msg/GenericTaskParams.msg"
  "msg/Task.msg"
  "msg/Position.msg"
  "msg/HumanTarget.msg"
  "msg/Tool.msg"
  "msg/BatteryControl.msg"
  "msg/MissionOver.msg"
  "msg/TaskQueue.msg"
)

set(ACTION_FILES
  "action/NewTask.action"
  "action/NewTaskList.action"
  "action/BatteryEnough.action"
  "action/RequestMobileChargingStation.action"
  "action/DoCloserInspection.action"
  "action/TaskResult.action"
  "action/HeuristicPlanning.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${ACTION_FILES}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    action_msgs
    sensor_msgs
    geographic_msgs
    as2_msgs
    as2_core
    builtin_interfaces
    ament_index_cpp
)

ament_export_dependencies(rosidl_default_runtime)

# -----------------------------------------------------------------------------
# Include directories
# -----------------------------------------------------------------------------
include_directories(
  include
)

# -----------------------------------------------------------------------------
# Core library
# -----------------------------------------------------------------------------
add_library(${PROJECT_NAME}_core src/classes.cpp)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  sensor_msgs
  geographic_msgs
  as2_msgs
  as2_core
  builtin_interfaces
)
if(behaviortree_cpp_v3_FOUND)
  ament_target_dependencies(${PROJECT_NAME}_core behaviortree_cpp_v3)
endif()
if(yaml-cpp_FOUND)
  target_link_libraries(${PROJECT_NAME}_core yaml-cpp)
endif()

# Link the generated interfaces the modern way, not rosidl_target_interfaces
# Get the typesupport target
rosidl_get_typesupport_target(c_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_c")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link the typesupport into the library
target_link_libraries(${PROJECT_NAME}_core ${c_typesupport_target})
target_link_libraries(${PROJECT_NAME}_core ${cpp_typesupport_target})

install(
  TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/mission_planner/
  DESTINATION include/mission_planner
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts
install(DIRECTORY
  scripts
  DESTINATION share/${PROJECT_NAME}/
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

install(PROGRAMS scripts/swarm_initializer.py scripts/mission_sequencer.py
             scripts/mission_viz.py scripts/gen_mission_labels.py
        DESTINATION lib/${PROJECT_NAME})

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# The world signs (models/mission_labels) are generated from
# config/mission.yaml and config/conf.yaml. Regenerated on every build so a
# change to the mission, or to the script constants, is picked up without
# having to remember to run the script by hand.
add_custom_target(mission_labels ALL
  COMMAND ${CMAKE_COMMAND} -E echo "Regenerating mission labels..."
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/gen_mission_labels.py
          || ${CMAKE_COMMAND} -E echo "WARNING: could not regenerate the mission labels"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "mission_labels: Gazebo signs and ground markings"
  VERBATIM)

install(
  DIRECTORY gazebo_worlds/
  DESTINATION share/${PROJECT_NAME}/
)

install(
  FILES 
    src/behaviour_tree.xml
  DESTINATION share/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
# Executables
# -----------------------------------------------------------------------------
function(add_agent_executable exe_name src_file)
  add_executable(${exe_name} src/${src_file})
  ament_target_dependencies(${exe_name}
    rclcpp
    rclcpp_action
    std_msgs
    geometry_msgs
    sensor_msgs
    as2_msgs
    geographic_msgs
    builtin_interfaces
    ament_index_cpp
  )
  target_link_libraries(${exe_name} ${PROJECT_NAME}_core)
  if(yaml-cpp_FOUND)
    target_link_libraries(${exe_name} yaml-cpp)
  endif()
  set_target_properties(${exe_name} PROPERTIES COMPILE_FLAGS "-g")
  install(TARGETS ${exe_name} DESTINATION lib/${PROJECT_NAME})
endfunction()

add_agent_executable(gesture_recognition_faker gesture_recognition_faker.cpp)
add_agent_executable(high_level_planner high_level_planner.cpp)
add_agent_executable(agent_behaviour_manager agent_behaviour_manager.cpp)
add_agent_executable(battery_faker battery_faker.cpp)
add_agent_executable(ist_ugv_faker ist_ugv_faker.cpp)
add_agent_executable(heuristic_planner_simulator heuristic_planner_simulator.cpp)

# -----------------------------------------------------------------------------
# Finalize
# -----------------------------------------------------------------------------
ament_package()