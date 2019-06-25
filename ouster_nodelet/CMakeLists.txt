cmake_minimum_required(VERSION 2.8.3)
project(ouster_nodelet)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11 -Wall -fPIC)

#############
# LIBRARIES #
#############

set(HEADER_FILES
    include/ouster_nodelet/os1_cloud_nodelet.h  
    include/ouster_nodelet/os1_assembler_nodelet.h
    )

set(SRC_FILES 
    src/os1_cloud_nodelet.cpp
    src/os1_assembler_nodelet.cpp
    )


cs_add_library(${PROJECT_NAME}
  ${SRC_FILES}
  ${HEADER_FILES}
)

############
# BINARIES #
############

cs_add_executable(os1_cloude_nodelet
  src/os1_cloud_node.cpp
)
target_link_libraries(os1_cloude_nodelet ${PROJECT_NAME})

cs_add_executable(os1_assembler_nodelet
  src/os1_assembler_node.cpp
)
target_link_libraries(os1_assembler_nodelet ${PROJECT_NAME})



##########
# EXPORT #
##########

install(
  FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cs_install()
cs_export()
