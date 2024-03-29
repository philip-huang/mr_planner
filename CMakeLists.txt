cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
project(mr_planner VERSION 1.0.0 LANGUAGES CXX)

# /* -------------------------------------------------------------------------- */
# /*                              Build Properties                              */
# /* -------------------------------------------------------------------------- */
add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

# /* -------------------------------------------------------------------------- */
# /*                                Find Package                                */
# /* -------------------------------------------------------------------------- */
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ompl REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  rviz_visual_tools
  moveit_visual_tools
  tf2_geometry_msgs 
)

# /* -------------------------------------------------------------------------- */
# /*                               Catkin Package                               */
# /* -------------------------------------------------------------------------- */
catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    tf2_geometry_msgs 
  DEPENDS
    EIGEN3
)

# /* -------------------------------------------------------------------------- */
# /*                               Include Directories                          */
# /* -------------------------------------------------------------------------- */

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

# /* -------------------------------------------------------------------------- */
# /*                               Create Library                               */
# /* -------------------------------------------------------------------------- */
file(GLOB src_files src/instance.cpp src/planner.cpp src/SingleAgentPlanner.cpp
                    src/logger.cpp src/tpg.cpp)

add_library(${PROJECT_NAME} SHARED ${src_files})

target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 1)

file(GLOB lego_files src/lego/*.cpp src/lego/Utils/*.cpp)

add_library(lego SHARED ${lego_files})

target_link_libraries(lego
  Eigen3::Eigen
  ${catkin_LIBRARIES}
  jsoncpp
)

target_include_directories(lego PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# /* -------------------------------------------------------------------------- */
# /*                             Testing Executables                            */
# /* -------------------------------------------------------------------------- */
add_executable(mr_planner_node
  src/plan_node.cpp
)


target_link_libraries(mr_planner_node
  ${PROJECT_NAME}
  lego
  ${catkin_LIBRARIES}
)

install(TARGETS mr_planner_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(demo
  src/demo_node.cpp
)


target_link_libraries(demo
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS demo
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(yk_robot
  src/yk_interface.cpp
)

target_include_directories(yk_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

target_link_libraries(yk_robot
  ${catkin_LIBRARIES}
)

install(TARGETS yk_robot
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)