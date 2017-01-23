#include "metaroom_xml_parser/load_utilities.h"
#include "semantic_data_store/util.h"


#include <nav_msgs/OccupancyGrid.h>
#include <ros/subscriber.h>
#include <soma_msgs/SOMAObject.h>
#include <deep_object_detection/DetectObjects.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <sys/types.h>
#include <sys/stat.h>



using namespace std;

tf::Vector3 sweepCenter;

Eigen::Vector4f robotPosition;

int main(int argc, char** argv)
{

    bool shouldVisualize = false; // change this if you want
    bool shouldStore = true;

    std::vector<string> labels;

    string dataPath = "";

    // Get the data path
    if (argc >= 2)
    {
        dataPath = argv[1];
    }
    else
    {
        cout<<"Please provide data path as argument"<<endl;
        return -1;
    }

    struct stat info;

    if( stat( dataPath.data(), &info ) != 0 ){
        cout<<"cannot access "<<dataPath<<". Quitting..."<<endl;
        return -2;
    }
    else if( info.st_mode & S_IFDIR )
        cout<<dataPath<<" is a directory"<<endl;
    else
    {
        cout<<dataPath<<" is not a directory. Quitting..."<<endl;
        return -3;
    }
    if(argc >= 3)
    {
        string shouldstore = argv[2];
        if(shouldstore == "0"){
            shouldStore = false;
            ROS_WARN("I won't store data!!");
        }

    }

    if(argc >= 4)
    {
        string shouldvisualize = argv[3];
        if(shouldvisualize == "1")
        {
            shouldVisualize = true;
            ROS_WARN("I will visualize data!!");
        }
    }


    /******* Initialize the ros node *******************/
    ros::init(argc, argv, "semantic_data_store");

    ros::NodeHandle n;

    //private node handle
    ros::NodeHandle pn("~");

    float confidence_threshold;
    pn.getParam("deep_net_confidence_threshold",confidence_threshold);

    // threshold to merge object with its neighbor in meters
    float object_neighbor_threshold;
    pn.getParam("object_neighbor_threshold",object_neighbor_threshold);

    // distance threshold of the object w.r.t the robot
    float object_distance_threshold;
    pn.getParam("object_distance_threshold",object_distance_threshold);

    // Get the labels from the parameter server otherwise it will use the chairs as the object of interest
    pn.getParam("object_labels",labels);

    ROS_INFO("Private params %.2f %.2f %.2f",confidence_threshold,object_neighbor_threshold,object_distance_threshold);

    /**************************************************/





    if(labels.size() == 0)
    {
        labels.push_back("chair");
        labels.push_back("person");
        ROS_WARN("No labels provided!! Detecting only chairs and people...");
    }
    else
    {
        ROS_INFO("Detecting following labels: ");
        for(auto label:labels)
        {
            ROS_INFO("%s ",label.data());
        }
    }

    // Wait for deep-net ros service, if not available return
    if(!ros::service::waitForService("/deep_object_detection/detect_objects",5))
    {
        ROS_ERROR("Deep Object Detection service not available. Quitting...");
        return -4;

    }



    ros::ServiceClient object_detection_service = n.serviceClient<deep_object_detection::DetectObjects>("/deep_object_detection/detect_objects");

    deep_object_detection::DetectObjects detect_objects;

    // Read the observations from XMLs
    vector<string> observations = semantic_map_load_utilties::getSweepXmls<PointType>(dataPath);

    //We now have the observation vector. Now we should parse the relevant data
    for(int i = 0; i < observations.size(); i++)
    {
        ROS_INFO("Observation path: %s",observations[i].data());

        RoomObservation roomobservation = Util::readRGBImagesfromRoomSweep(observations[i],sweepCenter);

        if(roomobservation.rosimagesclouds.size() > 0)
        {

            /************Transform images into ros images***********************/
            std::vector<sensor_msgs::Image > rosimages;

            std::vector<Cloud> clouds;

            for(int i = 0; i < roomobservation.rosimagesclouds.size(); i++)
            {

                rosimages.push_back(roomobservation.rosimagesclouds[i].first);
                clouds.push_back(roomobservation.rosimagesclouds[i].second);

            }
            /*******************************************************************/


            detect_objects.request.images = rosimages;
            detect_objects.request.observation_path = observations[i].data();
            detect_objects.request.confidence_threshold = 0.8;


            // If we can call the service and get a response
            if(object_detection_service.call(detect_objects))
            {
                // We refine the detected objects based on distance to not to include same object multiple times
                std::vector< std::pair<deep_object_detection::Object,Cloud> > refinedObjectsCloudsPair = Util::refineObjects(detect_objects.response.objects,clouds,rosimages[0].width,labels,sweepCenter,object_neighbor_threshold,object_distance_threshold);

                for(int j = 0; j < refinedObjectsCloudsPair.size(); j++)
                {

                    std::pair<deep_object_detection::Object,Cloud> apair = refinedObjectsCloudsPair[j];

                    RoomObject aroomobject;

                    aroomobject.object = apair.first;
                    aroomobject.cloud = apair.second;

                    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(roomobservation.rosimagesclouds[apair.first.imageID].first,"bgr8");


                    cv::rectangle(ptr->image,cv::Point(refinedObjectsCloudsPair[j].first.x,refinedObjectsCloudsPair[j].first.y),
                                  cv::Point(refinedObjectsCloudsPair[j].first.x+refinedObjectsCloudsPair[j].first.width, refinedObjectsCloudsPair[j].first.y + refinedObjectsCloudsPair[j].first.height),cv::Scalar(255,255,0));


                    cv::putText(ptr->image,refinedObjectsCloudsPair[j].first.label.data(),cv::Point(refinedObjectsCloudsPair[j].first.x+refinedObjectsCloudsPair[j].first.width*0.25,refinedObjectsCloudsPair[j].first.y+refinedObjectsCloudsPair[j].first.height*0.5),0,2.0,cv::Scalar(255,255,0));


                    ptr->toImageMsg(aroomobject.rosimage);

                    roomobservation.roomobjects.push_back(aroomobject);

                    if(shouldVisualize)
                    {

                        Util::visualizeDeepNetObjects(apair,roomobservation.rosimagesclouds[apair.first.imageID].first);
                        usleep(1000);
                    }

                }
                if(roomobservation.roomobjects.size() > 0 && shouldStore)
                   Util::logSOMAObjectsToDBCallService(n,roomobservation.room,roomobservation);
                roomobservation.roomobjects.clear();


            }

        }

        if(!ros::ok())
        {
            ROS_INFO("Breaking loop...");
            break;
        }
    }

    ROS_INFO("Finished inserting semantic data...");



    return 0;
}

