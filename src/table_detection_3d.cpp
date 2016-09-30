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

    ros::init(argc, argv, "table_detection_3d");

    ros::NodeHandle n;

    if(!ros::service::waitForService("detect_tables",5))
    {
          ROS_ERROR("Table detection service not available. Quitting...");
          return -1;

    }

    vector<string> observations = semantic_map_load_utilties::getSweepXmls<PointType>(dataPath);

  //  ros::ServiceClient table_detection_client = n.serviceClient<table_detection::DetectTables>("detect_tables");

    ros::Publisher pub = n.advertise<Cloud>("complete_clouds", 1);

    ros::Rate loop_rate(0.1);



    for(string observation:observations)
    {
        // Parse the room data
        //SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(observation,true);

        // Parse the necessary clouds
        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(observation, std::vector<std::string>{"RoomCompleteCloud","RoomIntermediateCloud"},false, true);


        Cloud msg ;

        msg.header.frame_id = "map";
        msg.height = sweep.completeRoomCloud->height;
        msg.width = sweep.completeRoomCloud->width;
       // ROS_INFO("Msg width: %ld height: %ld \n",msg.height,msg.width);
        msg.points.resize(msg.height*msg.width);
        msg.points = sweep.completeRoomCloud->points;
        msg.header.stamp = ros::Time::now().toSec();

        pcl_ros::transformPointCloud(msg, msg,sweep.vIntermediateRoomCloudTransforms[0]);


         pub.publish(msg);

         ros::spinOnce ();

         loop_rate.sleep ();

         if(!ros::ok())

             break;

       /* pcl_ros::transformPointCloud(msg, msg,sweep.vIntermediateRoomCloudTransforms[0]);


         table_detection::DetectTables detect_tables;

         sensor_msgs::PointCloud2 general_cloud_msg;

         pcl::toROSMsg(general_cloud, general_cloud_msg);

         detect_tables.request.pointcloud = general_cloud_msg;

         if(table_detection_client.call(detect_tables))
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
             }

             for(int i = 0; i < detect_tables.response.tables.size(); i++)
             {
                 strands_perception_msgs::Table aTable = detect_tables.response.tables[i];

                 tf::Pose tfpose;

                 tf::poseMsgToTF(aTable.pose.pose,tfpose);

                 tf::tfDistance2(tfpose.getOrigin())

                 aTable.pose.pose


             }

         }*/



    }



//    pcl::io::savePCDFileBinary("//home//hakan//results//complete_cloud.pcd",general_cloud);


    ROS_INFO("Table detection is done!");

    while (ros::ok())
    {
      //  pub.publish(general_cloud);

        ros::spinOnce ();
        loop_rate.sleep ();
    }



    return 0;
}
