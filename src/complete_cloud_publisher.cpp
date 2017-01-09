#include "mongodb_interface.h"
#include "util.h"
#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <semantic_map/semantic_map_summary_parser.h>
#include <semantic_map/room_xml_parser.h>
#include <ros/subscriber.h>
#include "table_detection/DetectTables.h"

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

    ros::init(argc, argv, "complete_cloud_publisher");

    ros::NodeHandle n;

    vector<string> observations = semantic_map_load_utilties::getSweepXmls<PointType>(dataPath);

    ros::Publisher pub = n.advertise<Cloud>("complete_clouds", 1);

    Cloud general_cloud;
    general_cloud.height  = 0;
    general_cloud.width = 1;
    general_cloud.header.frame_id = "map";


    for(string observation:observations)
    {


        // Parse the necessary clouds
        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(observation, std::vector<std::string>{"RoomCompleteCloud", "RoomIntermediateCloud"},false, true);

        // ros::init (argc, argv, "pub_pcl");
        // ros::NodeHandle nh;

        // Transform the local point cloud to map frame

        Cloud msg ;

        msg.header.frame_id = "map";
        msg.height = sweep.completeRoomCloud->height;
        msg.width = sweep.completeRoomCloud->width;
        ROS_INFO("Msg width: %ld height: %ld \n",msg.height,msg.width);
        msg.points.resize(msg.height*msg.width);
        msg.points = sweep.completeRoomCloud->points;

        pcl_ros::transformPointCloud(msg, msg,sweep.vIntermediateRoomCloudTransforms[0]);

        Cloud msgclamped = clampPointCloud("z",msg,0.0,1.5);


        general_cloud.height += msg.height;
       // general_cloud.width += msg.width;
        //general_cloud.points.resize(general_cloud.height*general_cloud.width);

        //general_cloud.points += msg.points;

       // pcl::concatenatePointCloud(msg,general_cloud,general_cloud);

        general_cloud +=msgclamped;



        //msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));



    }

    general_cloud.header.stamp = ros::Time::now().toSec();

    ros::Rate loop_rate(0.1);

    pub.publish(general_cloud);

  /*  if(!ros::service::waitForService("detect_tables",5))
    {
        ROS_ERROR("Table detection service not available. Quitting...");
        return -1;

    }*/

    pcl::io::savePCDFileBinary("//home//hakan//results//complete_cloud.pcd",general_cloud);


   /* ros::ServiceClient table_detection_client = n.serviceClient<table_detection::DetectTables>("detect_tables");

    table_detection::DetectTables detect_tables;

    sensor_msgs::PointCloud2 general_cloud_msg;

    pcl::toROSMsg(general_cloud, general_cloud_msg);

    detect_tables.request.pointcloud = general_cloud_msg;*/

   /* if(table_detection_client.call(detect_tables))
    {
        ROS_INFO("The number of detected tables %d",detect_tables.response.tables.size());

    }*/

    while (ros::ok())
    {
      //  pub.publish(general_cloud);

        ros::spinOnce ();
        loop_rate.sleep ();
    }



    return 0;
}
