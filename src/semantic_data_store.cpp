#include "../include/mongodb_interface.h"
#include "metaroom_xml_parser/load_utilities.h"
#include "util.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/subscriber.h>
#include <soma_map_manager/MapInfo.h>
#include <soma_msgs/SOMAOccupancyMap.h>
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

nav_msgs::OccupancyGrid globalmap;

bool mapAvailable = false;
bool shouldProceed = false;



ros::Subscriber mapsubscriber;




void mapCallback(nav_msgs::OccupancyGrid map)
{

    globalmap = map;

    // We got the map. We can unsubscribe from the topic
    mapsubscriber.shutdown();

    mapAvailable = true;

    ROS_INFO_STREAM("Map info received");
}

void timerCallback(const ros::TimerEvent& event)
{
    shouldProceed = true;
    if(!mapAvailable)
    {

        ROS_INFO_STREAM("Map message is not received!! Please check your map topic. Proceeding without map info...");
    }

}

void visualizeData( std::pair<deep_object_detection::Object,Cloud> apair, sensor_msgs::Image image)
{
    pcl::visualization::PCLVisualizer *p = new pcl::visualization::PCLVisualizer ("Labelled data");
    p->addCoordinateSystem();



    ROS_INFO("%s",apair.first.label.data());

    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(image,"bgr8");


    cv::imshow("image_org",ptr->image);


    cv::rectangle(ptr->image,cv::Point(apair.first.x,apair.first.y),
                  cv::Point(apair.first.x+apair.first.width, apair.first.y + apair.first.height),cv::Scalar(255,255,0));
    cv::putText(ptr->image,apair.first.label.data(),cv::Point(apair.first.x,apair.first.y-5),0,2.0,cv::Scalar(255,255,0));

    cv::imshow("image_found",ptr->image);
    cv::waitKey(0);
    cv::destroyAllWindows();



    //  pcl_ros::transformPointCloud(refinedObjectsCloudsPair[i].second, refinedObjectsCloudsPair[i].second,world_transform);


    std::stringstream ss;
    ss<<"cloud_"<<1;
    p->addPointCloud(apair.second.makeShared(),ss.str().data());




    p->spin();
    p->removeAllPointClouds();
    delete p;

    return;

}

/*std::vector< std::pair<deep_object_detection::Object,Cloud> > refineObjects(const std::vector<deep_object_detection::Object> &objects, const std::vector<Cloud> &clouds, int image_cols, const std::vector<std::string> labels)
{
    std::vector< std::pair<deep_object_detection::Object,Cloud> > result;

    std::vector<Cloud> remainingobjectclouds;

    std::vector<Eigen::Vector4f> remainingobjectcentroids;

    std::vector<deep_object_detection::Object> remainingobjects;

    //  Eigen::Vector4f sweepOrigin;

    //  roipc.width = pc.width;
    //  roipc.height = pc.height;
    //  roipc.resize(pc.width*pc.height);

    for(int i = 0; i < objects.size(); i++)
    {
        bool process = false;

        for(int j = 0; j < labels.size(); j++)
        {
            if (objects[i].label == labels[j])
            {
                process = true;
                break;
            }
        }

        if(process)
        {

            Cloud objectpc;

            deep_object_detection::Object obj = objects[i];

            for(int x = obj.x; x < obj.x + obj.width; x++)
            {
                for(int y = obj.y; y < obj.y + obj.height; y++)
                {

                    if(pcl_isfinite(clouds[obj.imageID].points[y*image_cols + x].x))

                        objectpc.push_back(clouds[obj.imageID].points[y*image_cols + x]);

                }
            }

            Eigen::Vector4f centroid;

            pcl::compute3DCentroid(objectpc,centroid);

            remainingobjectclouds.push_back(objectpc);

            remainingobjectcentroids.push_back(centroid);

            remainingobjects.push_back(obj);

        }
    }

    std::vector<bool> addindex;

    for(int i = 0; i < remainingobjectclouds.size();i++)
        addindex.push_back(true);

    for(int i = 0; i < remainingobjectclouds.size(); i++)
    {
        if(addindex[i])
        {

            for(int j = i+1 ; j < remainingobjectcentroids.size(); j++)
            {

                if(pcl::distances::l2(remainingobjectcentroids[i],remainingobjectcentroids[j]) <= 1.5)
                {
                    double isize = remainingobjects[i].width*remainingobjects[i].height;
                    double jsize = remainingobjects[j].width*remainingobjects[j].height;

                    std::cout<<pcl::distances::l2(remainingobjectcentroids[i],remainingobjectcentroids[j])<<std::endl;

                    if(remainingobjects[i].label == remainingobjects[j].label && isize < jsize)
                    {
                        //  addindex.push_back(i);
                        addindex[i] = false;
                        break;
                    }
                    else if(remainingobjects[i].label == remainingobjects[j].label)
                    {
                        addindex[j] = false;
                        //  break;
                    }
                    // addindex.push_back(j);

                }

            }
            // if(shouldadd)
            // addindex.push_back(i);
        }
    }

    // auto last = std::unique(addindex.begin(), addindex.end());
    // addindex.erase(last, addindex.end());

    for(int i =0; i < addindex.size(); i++)
    {
        Eigen::Vector4f vec4;
        vec4[0] = sweepCenter[0];
        vec4[1] = sweepCenter[1];
        vec4[2] = sweepCenter[2];

        double dist = pcl::distances::l2(remainingobjectcentroids[i],vec4);
        ROS_INFO("Distance of object w.r.t robot: %.2f",dist);

        if(addindex[i] == true && dist <= 6.0)
        {

            std::pair<deep_object_detection::Object,Cloud> objectcloudpair;

            objectcloudpair.first = remainingobjects[i];
            objectcloudpair.second = remainingobjectclouds[i];

            result.push_back(objectcloudpair);




           // Eigen::Vector3f vec3obj = remainingobjectcentroids[i].head<3>();


         //   ROS_INFO("Distance of object w.r.t robot: %.2f",pcl::distances::l2(remainingobjectcentroids[i],vec4));

         //   Eigen::Vector2f vec3 = robotPosition.head<2>();



            //  vec3 = vec3/vec3.squaredNorm()

         //   Eigen::Vector2f vec3obj = remainingobjectcentroids[i].head<2>();

        //    vec3obj -=vec3;

        //    Eigen::Vector2f zeros;

        //    zeros[0] = 1.0;
        //    zeros[1]=  0.0;
            //zeros[2] = 0.;

            // vec3.normalize();

        //    vec3obj.normalize();

        //    zeros.normalize();

         //   std::cout<<"Dots"<<" "<<(double)zeros.dot(vec3obj)<<" "<<acos((double)vec3obj.dot(zeros))<<" "<<vec3obj[0]<<" "<<vec3obj[1]<<" "<<std::endl;//vec3obj[2]<<std::endl;

        //    std::cout<<"Dots2"<<" "<<remainingobjects[i].width<<" "<<180*atan2(vec3obj[1],vec3obj[0])/3.14<<std::endl;



            //  ROS_INFO("angles %.2f",((float)180*acos((double)vec3.dot(vec3obj)/(vec3.squaredNorm()*vec3obj.squaredNorm()))/(M_PI)));
        }

    }



    return result;
}
*/

int main(int argc, char** argv)
{

    bool visualize = true; // change this if you want

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

    ros::init(argc, argv, "semantic_data_store");

    ros::NodeHandle n;

    MongodbInterface* m_MongodbInterface = new MongodbInterface(n);


    if(!ros::service::waitForService("/deep_object_detection/detect_objects",5))
    {
        ROS_ERROR("Deep Object Detection service not available. Quitting...");
        return -1;

    }

    ros::ServiceClient object_detection_service = n.serviceClient<deep_object_detection::DetectObjects>("/deep_object_detection/detect_objects");

    deep_object_detection::DetectObjects detect_objects;

    // Read the observations from XMLs
    vector<string> observations = semantic_map_load_utilties::getSweepXmls<PointType>(dataPath);

    //We now have the observation vector. Now we should parse the relevant data
    for(int i = 0; i < observations.size(); i++)
    {

        RoomObservation roomobservation = readRGBImagesfromRoomSweep(observations[i],sweepCenter);

        if(roomobservation.rosimagesclouds.size() > 0)
        {

            /***       Transform images into ros images      ****/
            std::vector<sensor_msgs::Image > rosimages;

            std::vector<Cloud> clouds;

            for(int i = 0; i < roomobservation.rosimagesclouds.size(); i++)
            {

                rosimages.push_back(roomobservation.rosimagesclouds[i].first);
                clouds.push_back(roomobservation.rosimagesclouds[i].second);

            }
            /*****************************************************/


            detect_objects.request.images = rosimages;
            detect_objects.request.confidence_threshold = 0.8;

            std::vector<string> labels;

            labels.push_back("chair");
           // labels.push_back("tvmonitor");
            //  labels.push_back("person");


            // If we can call the service and get a response
            if(object_detection_service.call(detect_objects))
            {
                // We refine the detected objects based on distance to not to include same object multiple times
                std::vector< std::pair<deep_object_detection::Object,Cloud> > refinedObjectsCloudsPair = refineObjects(detect_objects.response.objects,clouds,rosimages[0].width,labels,sweepCenter);

                for(int j = 0; j < refinedObjectsCloudsPair.size(); j++)
                {

                    std::pair<deep_object_detection::Object,Cloud> apair = refinedObjectsCloudsPair[j];

                    RoomObject aroomobject;

                    aroomobject.object = apair.first;
                    aroomobject.cloud = apair.second;

                    // aroomobject.image = roomobservation.rosimagesclouds[apair.first.imageID];

                    cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(roomobservation.rosimagesclouds[apair.first.imageID].first,"bgr8");


                    cv::rectangle(ptr->image,cv::Point(refinedObjectsCloudsPair[j].first.x,refinedObjectsCloudsPair[j].first.y),
                                  cv::Point(refinedObjectsCloudsPair[i].first.x+refinedObjectsCloudsPair[j].first.width, refinedObjectsCloudsPair[j].first.y + refinedObjectsCloudsPair[j].first.height),cv::Scalar(255,255,0));
                    cv::putText(ptr->image,refinedObjectsCloudsPair[j].first.label.data(),cv::Point(refinedObjectsCloudsPair[j].first.x,refinedObjectsCloudsPair[j].first.y-5),0,2.0,cv::Scalar(255,255,0));


                    ptr->toImageMsg(aroomobject.rosimage);

                    roomobservation.roomobjects.push_back(aroomobject);

                    if(visualize)
                        visualizeData(apair,roomobservation.rosimagesclouds[apair.first.imageID].first);

                }

                /*  pcl::visualization::PCLVisualizer *p = new pcl::visualization::PCLVisualizer (argc, argv, "Labelled data");
        p->addCoordinateSystem();

        for(int i = 0; i< refinedObjectsCloudsPair.size(); i++)
        {
            ROS_INFO("%s",refinedObjectsCloudsPair[i].first.label.data());

            cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(imagesandclouds[refinedObjectsCloudsPair[i].first.imageID].first,"bgr8");


            cv::rectangle(ptr->image,cv::Point(refinedObjectsCloudsPair[i].first.x,refinedObjectsCloudsPair[i].first.y),
                          cv::Point(refinedObjectsCloudsPair[i].first.x+refinedObjectsCloudsPair[i].first.width, refinedObjectsCloudsPair[i].first.y + refinedObjectsCloudsPair[i].first.height),cv::Scalar(255,255,0));
            cv::putText(ptr->image,refinedObjectsCloudsPair[i].first.label.data(),cv::Point(refinedObjectsCloudsPair[i].first.x,refinedObjectsCloudsPair[i].first.y-5),0,2.0,cv::Scalar(255,255,0));

            cv::imshow("image",ptr->image);
            cv::waitKey(0);
            cv::destroyAllWindows();



            //  pcl_ros::transformPointCloud(refinedObjectsCloudsPair[i].second, refinedObjectsCloudsPair[i].second,world_transform);


            std::stringstream ss;
            ss<<"cloud_"<<i;
            p->addPointCloud(refinedObjectsCloudsPair[i].second.makeShared(),ss.str().data());




        }

        p->spin();
        p->removeAllPointClouds();
        delete p;*/

              //  if(roomobservation.roomobjects.size() > 0)
                   // m_MongodbInterface->logSOMAObjectsToDBCallService(n,roomobservation.room,roomobservation);

            }

        }

        if(!ros::ok())
        {
            ROS_INFO("Breaking loop...");
            break;
        }
    }


    /*   pcl::visualization::PCLVisualizer *p = new pcl::visualization::PCLVisualizer (argc, argv, "Labelled data");
    p->addCoordinateSystem(); */


    /* vector<string> matchingObservations = semantic_map_load_utilties::getSweepXmlsForTopologicalWaypoint<PointType>(dataPath, waypId);




    SemanticRoomXMLParser<PointType> simple_parser;
    SemanticRoom<PointType> roomData;


    //std::vector<Entities> allSweeps = summary_parser.getRooms();

    ROS_INFO_STREAM("Observation matches for waypoint "<<matchingObservations.size()); */



    //MongodbInterface* map_MongodbInterface = new MongodbInterface(n);


    // ros::Timer timer = n.createTimer(ros::Duration(3), timerCallback);

    ros::Rate loop(10);

    // qDebug()<<"Waiting for map service from soma_map_manager";

    while(n.ok())
    {

        ros::spinOnce();
        loop.sleep();




    }





    /*  for (size_t i=0; i<matchingObservations.size();i++)
    {
        semantic_map_load_utilties::LabelledData<PointType> data = semantic_map_load_utilties::loadLabelledDataFromSingleSweep<PointType>(matchingObservations[i], false);

        if (data.objectClouds.size() == 0) continue; // no labelled objects


        // initialize ros
        // Set up ROS.
        ros::NodeHandle aRosNode("~");
        MongodbInterface* m_MongodbInterface = new MongodbInterface(aRosNode);

        cout<<"Parsing "<<matchingObservations[i]<<endl;
        SemanticRoom<PointType> aRoom = SemanticRoomXMLParser<PointType>::loadRoomFromXML(matchingObservations[i],true);


        static tf::StampedTransform world_transform = data.transformToGlobal;
        pcl_ros::transformPointCloud(*data.completeCloud, *data.completeCloud,world_transform);
        std::vector<tf::StampedTransform> tfs =  aRoom.getIntermediateCloudTransformsRegistered();
        int count = 0;
        for (auto object: data.objectClouds)
        {
            pcl_ros::transformPointCloud(*(object), *(object),world_transform);
            //      pcl_ros::transformPointCloud(*(object), *(object),tfs[data.objectScanIndices[count]]);

            if (visualize)
            {
                // pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_handler (data.completeCloud, 255, 0, 0);
                //  p->addPointCloud (data.completeCloud,cloud_handler,"complete_cloud");

                std::stringstream ss2;
                ss2<<"cloud_"<<count;
                p->addPointCloud(object,ss2.str());



            }

            count++;



            //  pcl_ros::transformPointCloud(*(object->m_points), *(object->m_points),world_transform);


        }

        // m_MongodbInterface->logRoomToDB(aRoom,matchingObservations[i]);

        // m_MongodbInterface->logObjectsToDB(aRoom,data,matchingObservations[i]);

        stringstream ss;
        ss<<"config"<<i;

        stringstream ss2;
        ss2<<i;

        // m_MongodbInterface->logSOMA2ObjectsToDB(aRoom,data,ss.str(),ss2.str(),srv.response.map_name.data(),srv.response.map_unique_id.data());

        m_MongodbInterface->logSOMA2ObjectsToDBCallService(n,aRoom,data,ss.str(),i,srv.response.map_name.data(),srv.response.map_unique_id.data());


       // std::string st;
        //  m_MongodbInterface->getMongoDBObjects(st);

        // ana haritaya gore merkezi almak icin
        //  data.transformToGlobal.getOrigin();


        cout<<"Now looking at "<<matchingObservations[i]<<"  acquisition time "<<data.sweepTime<<"   labelled objects  "<<data.objectClouds.size()<<endl;
        for ( size_t j=0; j<data.objectClouds.size(); j++ )
        {
            CloudPtr object = data.objectClouds[j];
            string label = data.objectLabels[j];
            cout<<"Labelled object "<<j<<"  points "<<object->points.size()<<"  label  "<<label<<endl;
            if (visualize)
            {
                stringstream ss;ss<<"object"<<j;
                //  p->addPointCloud(object,ss.str());
            }
        }

        if (visualize)
        {
            p->spin();
            p->removeAllPointClouds();
        }

    }*/
}

