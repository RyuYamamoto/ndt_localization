#ifndef _NDT_LOCALIZATION_
#define _NDT_LOCALIZATION_

#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pclomp/ndt_omp.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class NDTLocalization : public rclcpp::Node
{
  using PointType = pcl::PointXYZ;

public:
  NDTLocalization();
  ~NDTLocalization() = default;

private:
  void imu_callback(const sensor_msgs::msg::Imu & imu);
  void map_callback(const sensor_msgs::msg::PointCloud2 & map);
  void points_callback(const sensor_msgs::msg::PointCloud2 & points);
  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped & initialpose);
  void suggest_init_pose_callback(const geometry_msgs::msg::PoseStamped & suggest_init_pose);

  void downsample(
    const pcl::PointCloud<PointType>::Ptr & input_cloud_ptr,
    pcl::PointCloud<PointType>::Ptr & output_cloud_ptr);
  void crop(
    const pcl::PointCloud<PointType>::Ptr & input_cloud_ptr,
    pcl::PointCloud<PointType>::Ptr output_cloud_ptr);

  void publish_tf(
    const std::string frame_id, const std::string child_frame_id,
    const geometry_msgs::msg::PoseStamped pose);

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr suggest_init_pose_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ndt_align_cloud_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    ndt_pose_with_covariance_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr transform_probability_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ndt_path_publisher_;

  // ndt_omp
  std::shared_ptr<pclomp::NormalDistributionsTransform<PointType, PointType>> ndt_;

  geometry_msgs::msg::Pose initial_pose_;
  sensor_msgs::msg::Imu imu_data_;
  nav_msgs::msg::Path ndt_result_path_;

  tf2_ros::Buffer tf_buffer_{get_clock()};
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  std::deque<sensor_msgs::msg::Imu> imu_queue_;
  std::deque<geometry_msgs::msg::PoseStamped> pose_queue_;

  geometry_msgs::msg::Vector3 imu_velocity_;
  geometry_msgs::msg::Vector3 velocity_;

  bool correct_translation_offset_;
  bool correct_orientation_offset_;

  // config for ndt omp
  double transformation_epsilon_;
  double step_size_;
  double ndt_resolution_;
  int max_iteration_;
  int omp_num_thread_;
  std::string map_frame_id_;
  std::string base_frame_id_;

  double downsample_leaf_size_;

  double min_crop_vehicle_x_;
  double max_crop_vehicle_x_;
  double min_crop_vehicle_y_;
  double max_crop_vehicle_y_;

  double min_range_x_;
  double max_range_x_;
  double min_range_y_;
  double max_range_y_;
  double min_range_z_;
  double max_range_z_;

  bool localization_ready_{false};
};

#endif
