cmake_minimum_required(VERSION 2.4.6)

### If you're trying to debug include directives or so, this here
### might help:
#set (CMAKE_VERBOSE_MAKEFILE ON)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

### Debug mode implies slower runtime, "of course"
set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

# such that QT creator shows header files
file(GLOB_RECURSE HEADER_FILES include/*.h)

rosbuild_add_library(astarplanner src/AStarPlanner.cpp src/AStarPlannerROS.cpp ${HEADER_FILES})
