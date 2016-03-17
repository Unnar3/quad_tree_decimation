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

catkin_package(
	DEPENDS
	CATKIN_DEPENDS roscpp message_runtime qt_build ransac_primitives ProbabilisticPrimitiveRefinement
	INCLUDE_DIRS include
	LIBRARIES qtd_plane_extraction qtd_utils qtd_quad_tree qtd_quad_tree_pcl
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(qtd_plane_extraction src/plane_extraction.cpp)
target_link_libraries(qtd_plane_extraction ${catkin_LIBRARIES})

add_library(qtd_quad_tree src/quad_tree.cpp)
target_link_libraries(qtd_quad_tree ${catkin_LIBRARIES})

add_library(qtd_quad_tree_pcl src/quad_tree_pcl.cpp)
target_link_libraries(qtd_quad_tree_pcl qtd_quad_tree ${catkin_LIBRARIES})

add_library(qtd_utils src/utils.cpp)
target_link_libraries(qtd_utils ${catkin_LIBRARIES})

############################# INSTALL TARGETS

install(TARGETS qtd_plane_extraction qtd_utils qtd_quad_tree qtd_quad_tree_pcl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/quad_tree_decimation/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
