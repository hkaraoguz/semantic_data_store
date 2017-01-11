cmake_minimum_required(VERSION 2.8.3)
project(semantic_data_store)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs qt_build image_geometry tf pcl_ros mongodb_store semantic_map soma_msgs nav_msgs deep_object_detection cv_bridge soma_manager)



set(CMAKE_CXX_FLAGS "-O4 -g -fPIC -std=c++0x -fpermissive")

find_package(metaroom_xml_parser REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(qjson)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#link_directories(../../devel/lib/)
#include_directories(${PROJECT_DIR}/include/)
rosbuild_prepare_qt4(QtCore QtXml)

 add_message_files(
   FILES
   LabelledObject.msg
#   Message2.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES metaroom_xml_parser
   DEPENDS qt_build image_geometry libqt4-dev pcl_ros mongodb_store_modified semantic_map message_runtime soma_msgs soma_manager qjson
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../strands_3d_mapping/metaroom_xml_parser/include/
  ${OpenCV_HEADERS}
)

link_directories(${catkin_LINK_DIRS} ../../devel/lib/)

add_executable(semantic_data_store_node src/semantic_data_store.cpp src/util.cpp)

add_executable(complete_cloud_publisher src/complete_cloud_publisher.cpp src/util.cpp)

add_executable(table_detection_3d src/table_detection_3d.cpp src/util.cpp)



set_target_properties(semantic_data_store_node PROPERTIES LINKER_LANGUAGE CXX)

 target_link_libraries(semantic_data_store_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
   ${OpenCV_LIBRARIES}
   metaroom_xml_parser
   qjson
   jsoncpp
   )

target_link_libraries(complete_cloud_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${QT_LIBRARIES}
  ${OpenCV_LIBRARIES}
  metaroom_xml_parser
  qjson
  jsoncpp
)

target_link_libraries(table_detection_3d
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${QT_LIBRARIES}
  ${OpenCV_LIBRARIES}
  metaroom_xml_parser
  qjson
  jsoncpp
)

#############
# QTCREATOR #
#############
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers_SemanticDataStore SOURCES ${LibFiles})



############################# INSTALL TARGETS

install(TARGETS semantic_data_store_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/semantic_data_store/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
