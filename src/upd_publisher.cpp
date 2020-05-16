#include <ros/ros.h> 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
using namespace std;

// PCL Cloud type definition
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

/* This function receives the OpenNI data from a kinect device, 
    filter and classify the point cloud data with UPD classifier, 
    and publish the UPD classified data */
void process_upd(const PointCloud::ConstPtr& msg)
{
    // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    // BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    // printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("upd_point_cloud_classification", 1);
    pub.publish (msg);
    ros::spin();
}

int main(int argc, char** argv){
    
    // 1. ROS Init
    printf("Info: 1. ROS Init \n");
    ros::init (argc, argv, "upd_node");

    // 2. Subscriber OpenNI
    printf("Info: 2. Subscriber OpenNI \n");
    ros::NodeHandle nhk;

    // 3. Publishing UPD data 
    printf("Info: 3. Publishing UPD data \n");
    ros::Subscriber sub = nhk.subscribe<PointCloud>("/camera/depth/points", 1, process_upd);

    ros::spin();
}

// class UPD_Publisher{
//     public:
//         static const rosNode nh;
//         static const rosPublisher pub;
//         static const rosNode nhk;
//         static const rosSubscriber sub;
//         // UPD_Publisher(){
//         //     // 1. Setup UPD publisher that will provide classified points
//         //     pub = nh.advertise<PointCloud> ("upd_point_cloud_classification", 1);

//         //     // 2. Setup subscriber for Kinect Point Cloud data and run the callback process
//         //     sub = nhk.subscribe<PointCloud>("/camera/depth/points", 1, process);

//         //     // 3. ROS run
//         //     ros::Rate loop_rate(4);
//         //     ros::spin();

//         // }
//         static void process(const PointCloud::ConstPtr& msg){
//             // cout << "publisher callback processing.....";
//             printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//             BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//             printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

//             // 1. Publish the UPD data
//             //pub.publish (msg);
//         };

//         static void start(){

//             // 1. Setup UPD publisher that will provide classified points
//             pub = nh.advertise<PointCloud> ("upd_point_cloud_classification", 1);

//             // 2. Setup subscriber for Kinect Point Cloud data and run the callback process
//             sub = nhk.subscribe<PointCloud>("/camera/depth/points", 1, process);

//             // 3. ROS run
//             ros::Rate loop_rate(4);
//             ros::spin();

//         }

// };

// int main(int argc, char** argv){
    
//     ros::init (argc, argv, "upd_node");

//     UPD_Publisher upd_publisher;

//     upd_publisher.start();    

//     // upd_publisher();

// }