#include "semantic_data_store/mongodb_interface.h"
#include "semantic_data_store/util.h"
#include "metaroom_xml_parser/load_utilities.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <semantic_map/semantic_map_summary_parser.h>
#include <semantic_map/room_xml_parser.h>
#include <ros/subscriber.h>
#include <table_detection/DetectTables.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pwd.h>

const char *homedir;

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef typename Cloud::Ptr CloudPtr;

using namespace std;

int main(int argc, char** argv)
{

    string dataPath = "";
    bool save_cloud = true;
    float min_z = 0.0;
    float max_z = 1.5;
    bool transform_cloud = true;

    // Get the data path
    if (argc >= 2)
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

    //Private node handle to get node specific parameters
    ros::NodeHandle pnh("~");
    pnh.getParam("save_cloud",save_cloud);
    pnh.getParam("min_z",min_z);
    pnh.getParam("max_z",max_z);
    pnh.getParam("transform_cloud",transform_cloud);
    ROS_INFO("Params %d %d %.2f %.2f",save_cloud,transform_cloud,min_z,max_z);


    vector<string> observations = semantic_map_load_utilties::getSweepXmls<PointType>(dataPath);

    ros::Publisher pub = n.advertise<Cloud>("complete_clouds", 1);

    Cloud general_cloud;
    general_cloud.height  = 0;
    general_cloud.width = 1;
    general_cloud.header.frame_id = "map";


    // Publish the combination of clouds in a directory
    for(string observation:observations)
    {


        // Parse the necessary clouds
        auto sweep = SimpleXMLParser<PointType>::loadRoomFromXML(observation, std::vector<std::string>{"RoomCompleteCloud", "RoomIntermediateCloud"},false, true);


        Cloud msg ;

        msg.header.frame_id = "map";
        msg.height = sweep.completeRoomCloud->height;
        msg.width = sweep.completeRoomCloud->width;
        ROS_INFO("Msg width: %ld height: %ld \n",msg.height,msg.width);
        msg.points.resize(msg.height*msg.width);
        msg.points = sweep.completeRoomCloud->points;

         // Transform the local point cloud to map frame
        if(transform_cloud)
        {
            pcl_ros::transformPointCloud(msg, msg,sweep.vIntermediateRoomCloudTransforms[0]);
        }
        Cloud msgclamped = Util::clampPointCloud("z",msg,min_z,max_z);


        general_cloud.height += msg.height;

        general_cloud +=msgclamped;



    }

    general_cloud.header.stamp = ros::Time::now().toSec();



    pub.publish(general_cloud);


    if(save_cloud)
    {
        const char *homedir;

        if ((homedir = getenv("HOME")) == NULL) {
            homedir = getpwuid(getuid())->pw_dir;
        }
        std::string save_path(homedir);
        save_path +="//complete_cloud.pcd";
        ROS_INFO("Save path %s",save_path.data());
        pcl::io::savePCDFileBinary(save_path.data(),general_cloud);

    }
    ROS_INFO("Complete cloud has been constructed and published");


    return 0;
}
