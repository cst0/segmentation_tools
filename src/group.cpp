/*
 * ROS node that collects PointCloud2 messages and appends their data into one
 * message.
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>

ros::Publisher pub;
bool should_be_grouping = true;
pcl::PointCloud<pcl::PointXYZ> build_cloud;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input) {
  if (!should_be_grouping) {
      return;
  }
  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*input, cloud);

  build_cloud += cloud;
  build_cloud.header = cloud.header;

  // reduce number of points in build_cloud for performance
  int max_points = 100000;
    if (build_cloud.size() > max_points) {
        build_cloud.width = max_points;
        build_cloud.height = 1;
        build_cloud.points.resize(max_points);
    }

  // Convert the pcl/PointCloud to sensor_msgs/PointCloud2 and publish it
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(build_cloud, output);
  pub.publish(output);
}

bool start_grouping(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res) {
    should_be_grouping = true;
  res.success = true;
  res.message = "Grouping started";
  return true;
}

bool stop_grouping(std_srvs::Trigger::Request &req,
                   std_srvs::Trigger::Response &res) {
    should_be_grouping = false;
  res.success = true;
  res.message = "Grouping stopped";
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "group");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/segmented_cloud", 1, cloud_cb);

  // create a 'start grouping' service (using std_srvs/trigger)
  ros::ServiceServer start_grouping_service =
      nh.advertiseService("start_grouping", start_grouping);
  // create a 'stop grouping' service
  ros::ServiceServer stop_grouping_service =
      nh.advertiseService("stop_grouping", stop_grouping);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("/grouped_cloud", 1);

  // Spin
  ros::spin();
}
