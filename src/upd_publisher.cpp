#include <ros/ros.h> 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::ConstPtr msgNew;


void process_upd(const PointCloud::ConstPtr& msg)
{
    msgNew = msg;
    // cout << "publisher callback processing.....";
    printf ("Cloud: width = %d, height = %d\n", msgNew->width, msgNew->height);
    BOOST_FOREACH (const pcl::PointXYZ& pt, msgNew->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("upd_point_cloud_classification", 1);
    pub.publish (msgNew);
}

int main(int argc, char** argv){
    
    ros::init (argc, argv, "upd_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("upd_point_cloud_classification", 1);

    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "some_tf_frame";
    msg->height = msg->width = 1;
    msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));

    // Definig the node handler subscriber for kinect depth point cloud data
    cout << "Subscribing to the kinect point cloud node";
    ros::NodeHandle nhk;
    ros::Subscriber sub = nhk.subscribe<PointCloud>("/camera/depth/points", 1, process_upd);
    ros::spin();
    printf ("\t(%s)\n", "** calling the kinect **");

    // ros::Rate loop_rate(4);
    // while (nh.ok())
    // {
    //     printf ("\t message width: (%d)\n", msgNew->width);
    //     //pcl_conversions::toPCL(ros::Time::now(), msgNew->header.stamp);
    //     //pub.publish (msgNew);
    //     ros::spinOnce ();
    //     loop_rate.sleep ();
    // }
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