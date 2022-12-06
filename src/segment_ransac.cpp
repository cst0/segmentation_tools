/*
 * ROS node that takes in the point cloud from /camera/depth/points and segments
 * out the table and below leaving only the objects on the table. It then
 * publishes the segmented point cloud to /segmented_cloud
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
// include pcl_ros
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

// include tf2_ros
#include <tf2_ros/transform_listener.h>

ros::Publisher pub;
tf2_ros::Buffer tfBuffer;

// Finds the table plane in the point cloud using RANSAC
// Removes the table plane from the point cloud
// This function is called by cloud_cb
pcl::PointCloud<pcl::PointXYZ>::Ptr
remove_plane(const sensor_msgs::PointCloud2ConstPtr &input) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*input, *cloud);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 1.0);
  pass.filter(*cloud_filtered);

  // Create the segmentation object for the planar model and set all the
  // parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(
      new pcl::PointCloud<pcl::PointXYZ>());
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.01);

  int i = 0, nr_points = (int)cloud_filtered->points.size();

  // While 30% of the original cloud is still there
  // Segment the largest planar component from the remaining cloud

  float table_height = 0.0;
  while (cloud_filtered->points.size() > 0.3 * nr_points) {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      std::cout << "Could not estimate a planar model for the given dataset."
                << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);
    extract.setNegative(false);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    std::cout << "PointCloud representing the planar component: "
              << cloud_plane->points.size() << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);

    // update estimated table height
    table_height = coefficients->values[3];

    extract.filter(*cloud_filtered);
  }

  // delete all points under the table
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(
      new pcl::PointCloud<pcl::PointXYZ>);
  pass.setInputCloud(cloud_filtered);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(table_height, 1.0);
  pass.filter(*cloud_filtered2);

  // Return the point cloud with the table removed
  return cloud_filtered;
}

// Callback function for the point cloud subscriber
// This function is called every time a new point cloud is published
// to the /camera/depth/points topic
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // convert the frame of reference from the camera frame to the base_link
  // frame
  try {
    pcl_ros::transformPointCloud("base_link", *input, output, tfBuffer);
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    return;
  }

  // Remove the table plane from the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = remove_plane(input);
  // publish
  pub.publish(cloud_filtered);
}

// set up the ROS node
int main(int argc, char **argv) {
  ros::init(argc, argv, "segment");
  ros::NodeHandle nh;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("/segmented_cloud", 1);

  // Spin
  ros::spin();
}
