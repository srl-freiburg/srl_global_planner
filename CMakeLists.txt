cmake_minimum_required(VERSION 2.8.3)
project(srl_global_planner)

ADD_DEFINITIONS(-std=c++0x) # C++ 11 is required

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_core
  nav_msgs
  pluginlib
  costmap_2d
  roscpp
  rospy
  std_msgs
  tf
  visualization_msgs
)

find_package(cmake_modules REQUIRED)



# add qt4 to the project
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Boost REQUIRED)



catkin_package(

  CATKIN_DEPENDS
    geometry_msgs
    nav_core
    nav_msgs
    roscpp
    rospy
    std_msgs
    tf
    visualization_msgs
    costmap_2d
    pluginlib

  LIBRARIES
    srl_global_planner


)


FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
  message("OPENMP FOUND")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


###########
## Build ##
###########
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)


# RRT PLANNER

## Declare KdTree lib
add_library(SRLGLOBALPLANNER_KDTREE_LIB ${CMAKE_CURRENT_SOURCE_DIR}/include/smp/external_libraries/kdtree/kdtree.c)


# Setting SOURCES variables
set(SOURCES_RRT src/srl_global_planner.cpp include/srl_global_planner/srl_global_planner.h)

set(SOURCES
    src/thetastar_leading_rrt.cpp include/srl_global_planner/thetastar_leading_rrt.h include/srl_global_planner/costmap_model.cpp include/srl_global_planner/trajectory.cpp include/srl_global_planner/grid.cpp include/srl_global_planner/cell.cpp

)



# Plugin
add_library(srl_global_planner  ${SOURCES_RRT} ${SOURCES})

target_link_libraries(srl_global_planner ${catkin_LIBRARIES} ${BOOST_LIBRARIES} SRLGLOBALPLANNER_KDTREE_LIB)

add_dependencies(srl_global_planner srl_global_planner_gencfg ${catkin_EXPORTED_TARGETS})

add_dependencies(srl_global_planner nav_msgs_gencpp)


install(TARGETS srl_global_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}

       )

install(FILES srl_global_planner_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
