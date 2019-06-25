cmake_minimum_required(VERSION 2.8.3)
project(ouster_nodelet)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11 -Wall)

#
set(HEADER_FILES
    include/ouster_nodelet/OS1CloudNodelet.h  
    include/ouster_nodelet/assembler_nodelet.h
    )

set(SRC_FILES 
    src/OS1CloudNodelet.cpp
    src/assembler_nodelet.cpp
    )

cs_add_library(${PROJECT_NAME}
  ${SRC_FILES}
  ${HEADER_FILES}
)


##########
# EXPORT #
##########

install(
  FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cs_install()
cs_export()
