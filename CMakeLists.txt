cmake_minimum_required(VERSION 2.8.3)
project(srl_global_planner)
ADD_DEFINITIONS(-std=c++0x) # C++ 11 is required
# Adding definitions
MESSAGE(STATUS "Compiling with maximum optimization, will only work on Core i7 processors! (-mavx -march=corei7-avx -msse4.2 -O3 -funroll-loops -fomit-frame-pointer -ftree-vectorize ...)")
ADD_DEFINITIONS(-march=corei7-avx)
ADD_DEFINITIONS(-mtune=corei7-avx)
ADD_DEFINITIONS(-msse)
ADD_DEFINITIONS(-msse2)
ADD_DEFINITIONS(-msse3)
ADD_DEFINITIONS(-mssse3)
ADD_DEFINITIONS(-msse4.1)
ADD_DEFINITIONS(-msse4.2)
IF($ENV{AVX})
  ADD_DEFINITIONS(-mavx)
  MESSAGE(STATUS "Compiling with AVX on")
ELSE()
  MESSAGE(WARNING "Compiling with AVX off")
  ADD_DEFINITIONS(-mno-avx)
ENDIF()
ADD_DEFINITIONS(-O3)
ADD_DEFINITIONS(-funroll-loops)
ADD_DEFINITIONS(-fomit-frame-pointer)
ADD_DEFINITIONS(-pipe)
ADD_DEFINITIONS(-ftree-vectorize)
ADD_DEFINITIONS(-DNDEBUG)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_core
  nav_msgs
  spencer_tracking_msgs
  spencer_nav_msgs
  spencer_control_msgs
  pluginlib
  costmap_2d
  navfn
	global_planner
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
    navfn
		global_planner
    spencer_tracking_msgs
    spencer_nav_msgs
    spencer_control_msgs

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
    src/costmap_layers_dyn_rec_handler.cpp include/srl_global_planner/costmap_layers_dyn_rec_handler.h src/thetastar_leading_rrt.cpp include/srl_global_planner/thetastar_leading_rrt.h include/srl_global_planner/costmap_model.cpp include/srl_global_planner/trajectory.cpp include/srl_global_planner/grid.cpp include/srl_global_planner/cell.cpp

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
