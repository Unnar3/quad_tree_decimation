cmake_minimum_required(VERSION 2.8.3)
project(quad_tree_decimation)

set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x -fpermissive")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	pcl_conversions
  	pcl_ros
  	sensor_msgs
  	std_msgs
  	message_generation
    qt_build
    ProbabilisticPrimitiveRefinement
    ransac_primitives
)

rosbuild_prepare_qt4(QtCore QtXml)

#add_message_files(FILES
#)

#generate_messages(DEPENDENCIES
#	std_msgs
#	sensor_msgs
#)

find_package(CGAL REQUIRED COMPONENTS Core)

catkin_package(
	DEPENDS
	CATKIN_DEPENDS roscpp message_runtime qt_build
	INCLUDE_DIRS include
	LIBRARIES qtd_plane_extraction
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS})
include(${CGAL_USE_FILE})

add_library(qtd_plane_extraction src/plane_extraction.cpp)
target_link_libraries(qtd_plane_extraction ${catkin_LIBRARIES})

add_library(qtd_quad_tree src/quad_tree.cpp)
target_link_libraries(qtd_quad_tree ${catkin_LIBRARIES} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES} gmp)

add_library(qtd_quad_tree_pcl src/quad_tree_pcl.cpp)
target_link_libraries(qtd_quad_tree_pcl qtd_quad_tree ${catkin_LIBRARIES})

add_library(qtd_utils src/utils.cpp)
target_link_libraries(qtd_utils ${catkin_LIBRARIES})
