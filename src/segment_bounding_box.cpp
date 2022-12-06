/**
 * ROS node that
 * 1) gets a bounding box from the rosparam server
 * 2) gets a point cloud from a topic
 * 3) filters the point cloud to the bounding box
 * 4) publishes the filtered point cloud
 */

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

ros::Publisher pub;
ros::Publisher display_pub;
tf2_ros::Buffer tfBuffer;
std::vector<double> box; // TODO-- not actually using this
std::string reference_link;

// Callback function for the point cloud subscriber
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
  // transform cloud to be relative to the reference link
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *cloud);

  int original_size = cloud->size();
  // Create the filtering object
  pcl::CropBox<pcl::PointXYZ> boxFilter;
  boxFilter.setInputCloud(cloud);
    // TODO -- not actually using the input
  boxFilter.setMin(Eigen::Vector4f(-0.5, -0.5, -0.5, 1.0));
  boxFilter.setMax(Eigen::Vector4f(0.5, 0.5, 0.5, 1.0));
  boxFilter.filter(*cloud);

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);

  // Publish the data
  pub.publish(output);

  visualization_msgs::Marker marker;
  marker.header.frame_id = cloud->header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "segment_bounding_box";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 0.5;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration();
  marker.frame_locked = true;

  display_pub.publish(marker);
}

void populate_rosparams() {
  ros::NodeHandle nh;
  double min_x, min_y, min_z, max_x, max_y, max_z;
  nh.getParam("reference_link", reference_link);
  nh.getParam("min_x", min_x);
  nh.getParam("min_y", min_y);
  nh.getParam("min_z", min_z);
  nh.getParam("max_x", max_x);
  nh.getParam("max_y", max_y);
  nh.getParam("max_z", max_z);

  box = {min_x, min_y, min_z, max_x, max_y, max_z, 0, 0, 0, 0, 0, 0};
}

// set up the ROS node
int main(int argc, char **argv) {
  ROS_INFO("Starting segment_bounding_box node");
  ros::init(argc, argv, "segment");
  ros::NodeHandle nh;
  tf2_ros::TransformListener tfListener(tfBuffer);
  // get the bounding box from the rosparam server
  populate_rosparams();

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);
  display_pub =
      nh.advertise<visualization_msgs::Marker>("/segmented_parameters", 1);

  // Spin
  ros::spin();
}
