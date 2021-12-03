#ifndef _NDT_LOCALIZATION_
#define _NDT_LOCALIZATION_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>

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
#include <pcl_ros/transforms.hpp>

#include <pclomp/ndt_omp.h>

class NDTLocalization : public rclcpp::Node
{
  using PointType = pcl::PointXYZ;

public:
  NDTLocalization();
  ~NDTLocalization() = default;

private:
  void mapCallback(const sensor_msgs::msg::PointCloud2 & map);
  void pointsCallback(const sensor_msgs::msg::PointCloud2 & points);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped & initialpose);

  void downsample(
    const pcl::PointCloud<PointType>::Ptr & input_cloud_ptr,
    pcl::PointCloud<PointType>::Ptr & output_cloud_ptr);
  void crop(
    const pcl::PointCloud<PointType>::Ptr & input_cloud_ptr,
    pcl::PointCloud<PointType>::Ptr output_cloud_ptr, const double min_range,
    const double max_range);

  void publishTF(
    const std::string frame_id, const std::string child_frame_id,
    const geometry_msgs::msg::PoseStamped pose);

private:

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ndt_align_cloud_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr transform_probability_publisher_;

  // ndt_omp
  std::shared_ptr<pclomp::NormalDistributionsTransform<PointType, PointType>> ndt_;

  geometry_msgs::msg::Pose initial_pose_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  // config for ndt omp
  double transformation_epsilon_;
  double step_size_;
  double ndt_resolution_;
  int max_iteration_;
  int omp_num_thread_;
  std::string map_frame_id_;
  std::string base_frame_id_;

  double downsample_leaf_size_;

  double min_range_;
  double max_range_;

  bool localization_ready_{false};
};

#endif
