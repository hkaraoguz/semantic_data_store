#include "mongodb_interface.h"
#include "util.h"
#include "metaroom_xml_parser/load_utilities.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <semantic_map/semantic_map_summary_parser.h>
#include <semantic_map/room_xml_parser.h>
#include <ros/subscriber.h>

#include <table_detection/DetectTables.h>


#include <soma_msgs/SOMAObject.h>


#include <sys/types.h>
#include <sys/stat.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

using namespace std;


int main(int argc, char** argv)
{

    string dataPath = "";

    // Get the data path
    if (argc == 2)
    {
        dataPath = argv[1];
    } else {
        cout<<"Please provide data path as argument"<<endl;
        return -1;
    }

    struct stat info;

    if( stat( dataPath.data(), &info ) != 0 ){
        std::cout<<"cannot access "<<dataPath<<std::endl;
        return -2;
    }
    else if( info.st_mode & S_IFDIR )
        std::cout<<dataPath<<"is a directory"<<std::endl;
    else
    {
        std::cout<<dataPath<<"is no directory"<<std::endl;
        return -2;
    }

    ros::init(argc, argv, "table_detection_3d");

    ros::NodeHandle n;

    if(!ros::service::waitForService("detect_tables",60))
    {
        ROS_ERROR("Table detection service not available. Quitting...");
        return -1;

    }

    vector<string> observations = semantic_map_load_utilties::getSweepXmls<PointType>(dataPath);

      ros::ServiceClient table_detection_client = n.serviceClient<table_detection::DetectTables>("detect_tables");

    ros::Publisher pub = n.advertise<Cloud>("complete_clouds", 1);

    ros::Rate loop_rate(0.1);

    int id = 0;

    MongodbInterface* m_MongodbInterface = new MongodbInterface(n);

    for(string observation:observations)
    {
        // Parse the room data
        //SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(observation,true);

        /*
        struct RoomData
        {

            std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>>                            vIntermediateRoomClouds;
            std::vector<tf::StampedTransform>                                                     vIntermediateRoomCloudTransforms;
            std::vector<tf::StampedTransform>                                                     vIntermediateRoomCloudTransformsRegistered;
            std::vector<image_geometry::PinholeCameraModel>                                       vIntermediateRoomCloudCamParams;
            std::vector<image_geometry::PinholeCameraModel>                                       vIntermediateRoomCloudCamParamsCorrected;
            std::vector<cv::Mat>                                                                  vIntermediateRGBImages; // type CV_8UC3
            std::vector<cv::Mat>                                                                  vIntermediateDepthImages; // type CV_16UC1
            boost::shared_ptr<pcl::PointCloud<PointType>>                                         completeRoomCloud;
            boost::shared_ptr<pcl::PointCloud<PointType>>                                         dynamicClusterCloud;
            std::string                                                                           roomWaypointId;
            std::string                                                                           roomLogName;
            int                                                                                   roomRunNumber;
            boost::posix_time::ptime                                                              roomLogStartTime;
            std::vector<IntermediatePositionImages>                                               vIntermediatePositionImages;
            Eigen::Matrix4f                                                                       roomTransform;

            RoomData(){
                completeRoomCloud = boost::shared_ptr<pcl::PointCloud<PointType>>(new pcl::PointCloud<PointType>());
                dynamicClusterCloud = boost::shared_ptr<pcl::PointCloud<PointType>>(new pcl::PointCloud<PointType>());
        }

        */

        // Parse the necessary clouds
        SimpleXMLParser<PointType>::RoomData sweep = SimpleXMLParser<PointType>::loadRoomFromXML(observation, std::vector<std::string>{"RoomCompleteCloud","RoomIntermediateCloud"},false, true);



        Cloud msg ;

        msg.header.frame_id = "map";
        msg.height = sweep.completeRoomCloud->height;
        msg.width = sweep.completeRoomCloud->width;
        ROS_INFO("%s",sweep.completeRoomCloud->header.frame_id.data());
        // ROS_INFO("Msg width: %ld height: %ld \n",msg.height,msg.width);
        msg.points.resize(msg.height*msg.width);
        msg.points.assign(sweep.completeRoomCloud->points.begin(),sweep.completeRoomCloud->points.end());
        /* for(size_t i = 0; i< msg.points.size();i++)
        {
            PointType apoint = msg.points[i];
            ROS_INFO("x:%.2f y:%.2f z:%.2f",apoint.x,apoint.y,apoint.z);
        }*/


        msg.header.stamp = ros::Time::now().toSec();

        /*  pcl::PointCloud<pcl::PointXYZRGB> cloud = msg;

        pcl::PointXYZRGB min_pt;
        pcl::PointXYZRGB  max_pt;

        pcl::getMinMax3D(cloud,min_pt,max_pt)	;
        ROS_INFO("Min points x:%.2f, y:%.2f z:%.2f, Max points x:%.2f y:%.2f z:%.2f",min_pt.x,min_pt.y,min_pt.z,max_pt.x,max_pt.y,max_pt.z);*/


        //  pcl_ros::transformPointCloud(msg, msg,sweep.vIntermediateRoomCloudTransforms[0]);


        pub.publish(msg);

        ros::spinOnce ();

        loop_rate.sleep ();

        if(!ros::ok())

            break;

        /* pcl_ros::transformPointCloud(msg, msg,sweep.vIntermediateRoomCloudTransforms[0]);

*/
        table_detection::DetectTables detect_tables;

        sensor_msgs::PointCloud2 general_cloud_msg;

        pcl::toROSMsg(msg, general_cloud_msg);

        detect_tables.request.pointcloud = general_cloud_msg;

        /* if(table_detection_client.call(detect_tables))
         {
             ROS_INFO("The number of detected tables %d",detect_tables.response.tables.size());
             int i = 0;
             while ( i < detect_tables.response.tables.size()-1 )
             {
                 strands_perception_msgs::Table aTable = detect_tables.response.tables[i];
                 strands_perception_msgs::Table otherTable = detect_tables.response.tables[i+1];

                 tf::Pose currenttfpose;
                 tf::poseMsgToTF(aTable.pose.pose,currenttfpose);

                 tf::Pose othertfpose;
                 tf::poseMsgToTF(otherTable.pose.pose,othertfpose);


                 if ( shouldBeRemoved( currenttfpose,othertfpose,aTable. ) ) {
                     v.erase( v.begin() + i );
                 } else {
                     ++i;
                 }
             }*/
        if(table_detection_client.call(detect_tables))
        {
            std::vector<soma_msgs::SOMAObject> objects;
            for(int i = 0; i < detect_tables.response.tables.size(); i++)
            {
                strands_perception_msgs::Table aTable = detect_tables.response.tables[i];

                soma_msgs::SOMAObject object;

                object.metadata = convertTableData2Json(aTable);

                pcl::PointCloud<pcl::PointXYZ> obj_ros_cloud = crop3DObjectFromPointCloud(aTable,msg);

                pcl::toROSMsg(obj_ros_cloud, object.cloud);

                object.pose = aTable.pose.pose;

                objects.push_back(object);




            }

            if(objects.size() > 0)
                m_MongodbInterface->logTableDetectionBasedSOMAObjects<PointType>(n,sweep, objects);



        }



    }



    //    pcl::io::savePCDFileBinary("//home//hakan//results//complete_cloud.pcd",general_cloud);


    ROS_INFO("Table detection is done!");


    return 0;
}
