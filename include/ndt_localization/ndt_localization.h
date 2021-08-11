#ifndef _NDT_LOCALIZATION_
#define _NDT_LOCALIZATION_

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pclomp/ndt_omp.h>

class NDTLocalization
{
  using PointType = pcl::PointXYZ;

public:
  NDTLocalization();
  ~NDTLocalization() = default;

private:
  void mapCallback(const sensor_msgs::PointCloud2 & map);
  void pointsCallback(const sensor_msgs::PointCloud2 & points);
  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped & initialpose);

  void downsample(
    const pcl::PointCloud<PointType>::Ptr & input_cloud_ptr,
    pcl::PointCloud<PointType>::Ptr & output_cloud_ptr);

  void publishTF(
    const std::string frame_id, const std::string child_frame_id,
    const geometry_msgs::PoseStamped pose);

private:
  ros::NodeHandle nh_{};
  ros::NodeHandle pnh_{"~"};

  ros::Subscriber map_subscriber_;
  ros::Subscriber points_subscriber_;
  ros::Subscriber initialpose_subscriber_;
  ros::Publisher ndt_align_cloud_publisher_;
  ros::Publisher ndt_pose_publisher_;
  ros::Publisher transform_probability_publisher_;

  // ndt_omp
  boost::shared_ptr<pclomp::NormalDistributionsTransform<PointType, PointType>> ndt_;

  geometry_msgs::Pose initial_pose_;

  tf2_ros::TransformBroadcaster broadcaster_;

  // config for ndt omp
  double transformation_epsilon_;
  double step_size_;
  double ndt_resolution_;
  int max_iteration_;
  int omp_num_thread_;
  std::string map_frame_id_;
  std::string base_frame_id_;

  double downsample_leaf_size_;

  bool localization_ready_{false};
};

#endif
