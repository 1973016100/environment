#include <ros/ros.h>
#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <algorithm>

#include <nav_msgs/Odometry.h>
 #include <sensor_msgs/Imu.h>
 #include <tf/transform_datatypes.h>
 #include <tf/transform_broadcaster.h>
 #include "ground_extract.h"
#include "SFND_Lidar/point_flag.h"
#include <chrono>

//#include<pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace lidar_obstacle_detection;
using namespace chrono;



ros::Publisher pubLaserCloudSurround;
//Stream cityBlock function


ProcessPointClouds<pcl::PointXYZI> *pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn3(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn4;
pcl::PointCloud<pcl::PointXYZ> cloud55;

bool newCloud;

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZI> *pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud) {
    // Setting hyper parameters
    // FilterCloud
    float filterRes = 0.5;//0.4
    //Eigen::Vector4f minpoint(-60, -60.5, -1.5, 1);
    //Eigen::Vector4f maxpoint(60, 60.5, 2, 1);

   Eigen::Vector4f minpoint(-10, -10.5, -2, 1);
    Eigen::Vector4f maxpoint(10, 10.5, 2, 1);

    //Eigen::Vector4f minpoint(-40, -40.5, -5, 1);
    //Eigen::Vector4f maxpoint(40, 40.5, 5, 1);
    // SegmentPlane
    int maxIterations = 40;
    float distanceThreshold = 0.3;
    // Clustering
    float clusterTolerance = 0.5;
    int minsize = 10;
    int maxsize = 140;
	std::cout << "filter before      "<<inputCloud->size()<<std::endl;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minpoint,maxpoint);
    std::cout << "filter after       "<<filteredCloud->size()<<std::endl;
    /*--------------------新分割------------------------*/
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacSegmentPlane(
           filteredCloud, maxIterations, distanceThreshold);
    /*-------------------------------------------------*/
    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0));
    //renderPointCloud(viewer,inputCloud,"inputCloud");
    
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->EuclideanClustering(segmentCloud.first,
                                                                                                  clusterTolerance,
                                                                                                  minsize, maxsize);
    pcl::PointCloud<pcl::PointXYZI> lasersurround;
    //std::vector<SFND_Lidar::point_flag> 
    SFND_Lidar::point_flag laserpoint_with_long;
    SFND_Lidar::point_flag point_send;
    std::cout << "segmentCloudsecond size"<<"  "<<segmentCloud.second->points.size()<<std::endl;

    for(int counter = 0;counter < segmentCloud.second->points.size() ;counter ++){
        //segmentCloud.second->points[counter].intensity = 0;
        segmentCloud.second->points[counter].intensity = 0;
        //laserpoint_with_long.push_back(point_send);
        lasersurround.push_back(segmentCloud.second->points[counter]);
    }


    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};
    
    std::cout << "clusters size"<<"  "<<cloudClusters.size()<<std::endl;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters) 
  
    for (int i = 0;i < cloudClusters.size(); i++){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster = cloudClusters[i];
        Box box = pointProcessorI->BoundingBox(cluster);
        //std::cout << "box xyz min max "<<"  "<<box.x_min<<std::endl;
        //std::cout << "cluster size"<<"  "<<cluster->points.size();
        
        
        for(int counter = 0;counter < cluster->points.size() ;counter ++){
                
                cluster->points[counter].intensity = i+1;
                /*point_send->points = cluster->points[counter];
                point_send.x_min = box.x_min;
                point_send.y_min = box.y_min;
                point_send.z_min = box.z_min;
                */
                lasersurround.push_back(cluster->points[counter]);
            }


        //std::cout << "the   "<< i<<"    clusters"<<" xyz min max "<<cluster->points[0].x<<"  "<< cluster->points[1].x<<std::endl;
        
        //pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCLoud" + std::to_string(clusterId),colors[clusterId % colors.size()]);
        // Fourth: Find bounding boxes for each obstacle cluster
       
        renderBox(viewer, box, clusterId);
        ++clusterId;

    }
    
    sensor_msgs::PointCloud2 laserCloudSurround3;
    pcl::toROSMsg(lasersurround, laserCloudSurround3);
    laserCloudSurround3.header.stamp = ros::Time().now();
    laserCloudSurround3.header.frame_id = "camera_init";
    pubLaserCloudSurround.publish(laserCloudSurround3);
    
}



void sevenlidarlaserCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloudIn2)
{
  	//timeScanCurLast = timeScanCur;
 	//timeScanCur = laserCloudIn2->header.stamp.toSec();
  	
	//pcl::PointCloud<pcl::PointXYZI> temp;
	pcl::fromROSMsg(*laserCloudIn2, *laserCloudIn3);//*laserCloudIn3
    std::vector<int> index;
    laserCloudIn3->is_dense = false;
    pcl::removeNaNFromPointCloud(*laserCloudIn3,*laserCloudIn3,index);


	//*laserCloudIn3= temp;
	//*laserCloudIn4 = *laserCloudIn3;
	//std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(*laserCloudIn3,*laserCloudIn3,indices);
   	newCloud = true;
/*
			if(!viewer->wasStopped())
            {
                newCloud = false;
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();
                // Load pcd and run obstacle detection process
                //inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

                ///auto start = system_clock::now();
                //std::cout << "cloud33333333"<<"  "<<laserCloudIn3->size()<<std::endl;

                inputCloudI = pointProcessorI->loadPcd("/home/robot/zzw_ws/src/SFND_Lidar/src/sensors/data/pcd/data_1/0000000022.pcd");

                ///auto end = system_clock::now();
                ///auto time = duration<double>(end - start);
                ///std::cout << "single pcd time"<<"  "<<time.count()*1000<<std::endl;

                cityBlock(viewer, pointProcessorI, laserCloudIn3);//inputCloudI   laserCloudIn3
                viewer->spinOnce();
            }*/

            if(!viewer->wasStopped() && newCloud)
            {
                newCloud = false;
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();
                //inputCloudI = pointProcessorI->loadPcd("/home/robot/zzw_ws/src/SFND_Lidar/src/sensors/data/pcd/data_1/0000000022.pcd");
                cityBlock(viewer, pointProcessorI, laserCloudIn3);//inputCloudI   laserCloudIn3
                viewer->spinOnce();
            }



}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer) {

    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle) {
        case XY :
            viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
            break;
        case TopDown :
            viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
            break;
        case Side :
            viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
            break;
        case FPS :
            viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

// char* argv[] means array of char pointers, whereas char** argv means pointer to a char pointer.
int main(int argc, char **argv) {


    ros::init(argc,argv,"environmentnode");
    ros::NodeHandle n;
    std::cout << "-------------------------------" << std::endl;
    std::cout << "starting enviroment" << std::endl;

    
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    //std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("/home/robot/zzw_ws/src/SFND_Lidar/src/sensors/data/pcd/data_1");
    //std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("/home/robot/zzw_ws/src/SFND_Lidar/src/sensors/data/pcd/data_1");
    //auto streamIterator = stream.begin();
    
    ros::Subscriber subLaserCloud = n.subscribe<sensor_msgs::PointCloud2>("/7lidar", 10, sevenlidarlaserCloudHandler);
    pubLaserCloudSurround = n.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);

  	ros::Rate loop_rate(100);
    int count = 0;
    while(ros::ok())
    {
       /*
            if(!viewer->wasStopped() && newCloud)
            {
                newCloud = false;
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();
                // Load pcd and run obstacle detection process
                //inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());

                ///auto start = system_clock::now();
                

                //inputCloudI = pointProcessorI->loadPcd("/home/robot/zzw_ws/src/SFND_Lidar/src/sensors/data/pcd/data_1/0000000022.pcd");

                ///auto end = system_clock::now();
                ///auto time = duration<double>(end - start);
                ///std::cout << "single pcd time"<<"  "<<time.count()*1000<<std::endl;

                cityBlock(viewer, pointProcessorI, laserCloudIn3);//inputCloudI   laserCloudIn3
                viewer->spinOnce();
            }
        */

/*
            if(!viewer->wasStopped() && newCloud)
            {
                newCloud = false;
                viewer->removeAllPointClouds();
                viewer->removeAllShapes();
                //inputCloudI = pointProcessorI->loadPcd("/home/robot/zzw_ws/src/SFND_Lidar/src/sensors/data/pcd/data_1/0000000022.pcd");
                cityBlock(viewer, pointProcessorI, laserCloudIn3);//inputCloudI   laserCloudIn3
                viewer->spinOnce();
            }
      */ 



		ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}