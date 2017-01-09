#ifndef UTIL_H
#define UTIL_H

#include "metaroom_xml_parser/load_utilities.h"
#include <semantic_map/semantic_map_summary_parser.h>
#include <semantic_map/room_xml_parser.h>
#include <deep_object_detection/DetectObjects.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <tf/tf.h>
#include <pcl/visualization/cloud_viewer.h>
#include <jsoncpp/json/json.h>
#include <geometry_msgs/Polygon.h>
#include <strands_perception_msgs/Table.h>


typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;
typedef pcl::search::KdTree<PointType> Tree;
typedef typename SemanticMapSummaryParser::EntityStruct Entities;
typedef strands_perception_msgs::Table Table;

// This is the RoomObject struct that contains the ros image, cloud and the deep_net object information
struct RoomObject
{
    sensor_msgs::Image rosimage;
    Cloud cloud;
    deep_object_detection::Object object;

};

// This is the Room Observation class that contains the room information vector of ros images and clouds and the room objects
class RoomObservation
{
public:

    SemanticRoom<PointType> room;

    std::vector< std::pair< sensor_msgs::Image, Cloud > > rosimagesclouds;

    std::vector<RoomObject> roomobjects;
};


class Util
{
public:

    Cloud clampPointCloud(const std::string dimension, Cloud cloud, float maxrange);

    RoomObservation readRGBImagesfromRoomSweep(const std::string &observationpath, tf::Vector3 &robotPosition);


    Cloud crop3DObjectFromPointCloud(const deep_object_detection::Object& object, Cloud objectcloud, int image_cols);

    

  std::vector< std::pair<deep_object_detection::Object,Cloud> > refineObjects(const std::vector<deep_object_detection::Object> &objects, const std::vector<Cloud> &clouds,

                                                                            int image_cols, const std::vector<std::string> labels, tf::Vector3 robotPosition);

  std::string convertGeometryMsgsPolygon2Json(const geometry_msgs::Polygon& polygon, const std::string &key);

  std::string convertGeometryMsgsPose2Json(const geometry_msgs::Pose& pose);

  std::string convertTableData2Json(const Table& table);



    void visualizeDeepNetObjects( std::pair<deep_object_detection::Object,Cloud> apair, sensor_msgs::Image image);

    bool logSOMAObjectsToDBCallService(ros::NodeHandle n, SemanticRoom<PointType> aRoom, RoomObservation obs );

};


#endif // UTIL_H
