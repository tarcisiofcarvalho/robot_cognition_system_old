#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
  printf ("\t(%s)\n", "** ROS Init **");
  ros::init(argc, argv, "sub_upd");
  ros::NodeHandle nh;
  printf ("\t(%s)\n", "** UPD callback **");
  ros::Subscriber sub = nh.subscribe<PointCloud>("upd_point_cloud_classification", 1, callback);
  printf ("\t(%s)\n", "** UPD Callback finished **");
  ros::spin();
}