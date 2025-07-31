cmake_minimum_required(VERSION 3.5)
project(mrs_llcp_ros)

# set the correct stadards
set(CMAKE_C_STANDARD 99) 
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)
if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_autostart]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

add_definitions(-DLLCP_LITTLE_ENDIAN)

# find packages
set (DEPENDENCIES 
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  mrs_lib
  mrs_modules_msgs
  mrs_msgs
  mrs_serial
  )

set (LIBRARIES
  MrsLlcpRos_Llcp
  )

set (EXECUTABLES
  MrsLlcpRos_LlcpNode
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  mrs_llcp
  ${rclcpp_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  ${mrs_serial_INCLUDE_DIRS}
)

## -------------------------------------------------------------------------------------------------
## |                           Compile                                                             |
## -------------------------------------------------------------------------------------------------

## | ----------------------- llcp ros node ------------------------------------------------------- |

add_executable(MrsLlcpRos_LlcpNode
  src/llcp_ros_node.cpp
  src/llcp_ros.cpp
  mrs_llcp/llcp.c
  )

ament_target_dependencies(MrsLlcpRos_LlcpNode
  ${DEPENDENCIES}
  )

## | ----------------------- llcp ros lib ------------------------------------------------------- |

add_library(MrsLlcpRos_Llcp SHARED
  src/llcp_ros.cpp
  mrs_llcp/llcp.c
  )

ament_target_dependencies(MrsLlcpRos_Llcp
  ${DEPENDENCIES}
  )

target_compile_definitions(MrsLlcpRos_Llcp PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

rclcpp_components_register_nodes(MrsLlcpRos_Llcp PLUGIN "MrsLlcpRos" EXECUTABLE mrs_llcp_node)


## -------------------------------------------------------------------------------------------------
## |                           install                                                             |
## -------------------------------------------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
  )

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(TARGETS ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  )

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
  )

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
