/*
 * ROS node that converts one segmented cloud into multiple clouds, one for each
 * segment.
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*input, cloud);

  // Convert the pcl/PointCloud to sensor_msgs/PointCloud2 and publish it
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  pub.publish(output);
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "chunk_segments");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("segmented_cloud", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("chunked_segments", 1);

  // Spin
  ros::spin();
}
