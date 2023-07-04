#include <ndt_localization/ndt_localization.h>

//#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>

NDTLocalization::NDTLocalization() : Node("ndt_localization")
{
  min_range_ = this->declare_parameter("min_range", 0.5);
  max_range_ = this->declare_parameter("max_range", 60.0);
  downsample_leaf_size_ = this->declare_parameter("downsample_leaf_size", 3.0);
  transformation_epsilon_ = this->declare_parameter("transformation_epsilon", 0.01);
  step_size_ = this->declare_parameter("step_size", 0.1);
  ndt_resolution_ = this->declare_parameter("ndt_resolution", 5.0);
  max_iteration_ = this->declare_parameter("max_iteration", 20);
  omp_num_thread_ = this->declare_parameter("omp_num_thread", 3);
  map_frame_id_ = this->declare_parameter("map_frame_id", "map");
  base_frame_id_ = this->declare_parameter("base_frame_id", "base_link");

  ndt_ = std::make_shared<pclomp::NormalDistributionsTransform<PointType, PointType>>();
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  ndt_->setTransformationEpsilon(transformation_epsilon_);
  ndt_->setStepSize(step_size_);
  ndt_->setResolution(ndt_resolution_);
  ndt_->setMaximumIterations(max_iteration_);
  ndt_->setNeighborhoodSearchMethod(pclomp::KDTREE);
  if (0 < omp_num_thread_) ndt_->setNumThreads(omp_num_thread_);

  map_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&NDTLocalization::mapCallback, this, std::placeholders::_1));
  points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(5),
    std::bind(&NDTLocalization::pointsCallback, this, std::placeholders::_1));
  initialpose_subscriber_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1,
      std::bind(&NDTLocalization::initialPoseCallback, this, std::placeholders::_1));

  ndt_align_cloud_publisher_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("aligned_cloud", 10);
  ndt_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_publisher_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "ndt_pose_with_covariance", 10);
  transform_probability_publisher_ =
    this->create_publisher<std_msgs::msg::Float32>("transform_probability", 10);
}

void NDTLocalization::downsample(
  const pcl::PointCloud<PointType>::Ptr & input_cloud_ptr,
  pcl::PointCloud<PointType>::Ptr & output_cloud_ptr)
{
  pcl::VoxelGrid<PointType> voxel_grid;
  voxel_grid.setLeafSize(downsample_leaf_size_, downsample_leaf_size_, downsample_leaf_size_);
  voxel_grid.setInputCloud(input_cloud_ptr);
  voxel_grid.filter(*output_cloud_ptr);
}

void NDTLocalization::crop(
  const pcl::PointCloud<PointType>::Ptr & input_cloud_ptr,
  pcl::PointCloud<PointType>::Ptr output_cloud_ptr, const double min_range, const double max_range)
{
  for (const auto & p : input_cloud_ptr->points) {
    const double dist = std::sqrt(p.x * p.x + p.y * p.y);
    if (min_range < dist && dist < max_range) {
      output_cloud_ptr->points.emplace_back(p);
    }
  }
}

void NDTLocalization::mapCallback(const sensor_msgs::msg::PointCloud2 & map)
{
  RCLCPP_INFO(get_logger(), "map callback");

  pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(map, *map_cloud);

  ndt_->setInputTarget(map_cloud);
}

void NDTLocalization::pointsCallback(const sensor_msgs::msg::PointCloud2 & points)
{
  if (ndt_->getInputTarget() == nullptr) {
    RCLCPP_ERROR(get_logger(), "map not received!");
    return;
  }

  if (!localization_ready_) {
    RCLCPP_ERROR(get_logger(), "initial pose not received!");
    return;
  }

  const rclcpp::Time current_scan_time = points.header.stamp;

  pcl::PointCloud<PointType>::Ptr input_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(points, *input_cloud_ptr);

  // downsampling input point cloud
  pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
  downsample(input_cloud_ptr, filtered_cloud);

  // crop point cloud
  pcl::PointCloud<PointType>::Ptr crop_cloud(new pcl::PointCloud<PointType>);
  crop(filtered_cloud, crop_cloud, min_range_, max_range_);

  crop_cloud->width = crop_cloud->points.size();
  crop_cloud->height = 1;

  // transform base_link to sensor_link
  pcl::PointCloud<PointType>::Ptr transform_cloud_ptr(new pcl::PointCloud<PointType>);
  const std::string sensor_frame_id = points.header.frame_id;
  geometry_msgs::msg::TransformStamped sensor_frame_transform;
  try {
    sensor_frame_transform = tf_buffer_.lookupTransform(
      base_frame_id_, sensor_frame_id, current_scan_time, tf2::durationFromSec(0.5));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
    sensor_frame_transform.header.stamp = current_scan_time;
    sensor_frame_transform.header.frame_id = base_frame_id_;
    sensor_frame_transform.child_frame_id = sensor_frame_id;
    sensor_frame_transform.transform.translation.x = 0.0;
    sensor_frame_transform.transform.translation.y = 0.0;
    sensor_frame_transform.transform.translation.z = 0.0;
    sensor_frame_transform.transform.rotation.w = 1.0;
    sensor_frame_transform.transform.rotation.x = 0.0;
    sensor_frame_transform.transform.rotation.y = 0.0;
    sensor_frame_transform.transform.rotation.z = 0.0;
  }
  const Eigen::Affine3d base_to_sensor_frame_affine = tf2::transformToEigen(sensor_frame_transform);
  const Eigen::Matrix4f base_to_sensor_frame_matrix =
    base_to_sensor_frame_affine.matrix().cast<float>();
  pcl::transformPointCloud(*crop_cloud, *transform_cloud_ptr, base_to_sensor_frame_matrix);
  ndt_->setInputSource(transform_cloud_ptr);

  // calculation initial pose for NDT
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  Eigen::Affine3d initial_pose_affine;
  tf2::fromMsg(initial_pose_, initial_pose_affine);
  init_guess = initial_pose_affine.matrix().cast<float>();

  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  ndt_->align(*output_cloud, init_guess);

  const bool convergenced = ndt_->hasConverged();

  const Eigen::Matrix4f result_ndt_pose = ndt_->getFinalTransformation();

  Eigen::Affine3d result_ndt_pose_affine;
  result_ndt_pose_affine.matrix() = result_ndt_pose.cast<double>();
  const geometry_msgs::msg::Pose ndt_pose = tf2::toMsg(result_ndt_pose_affine);
  initial_pose_ = ndt_pose;

  geometry_msgs::msg::PoseStamped ndt_pose_msg;
  ndt_pose_msg.header.frame_id = map_frame_id_;
  ndt_pose_msg.header.stamp = current_scan_time;
  ndt_pose_msg.pose = ndt_pose;

  // TODO: replace covariance from hessian matrix
  geometry_msgs::msg::PoseWithCovarianceStamped ndt_pose_with_covariance_msg;
  ndt_pose_with_covariance_msg.header.frame_id = map_frame_id_;
  ndt_pose_with_covariance_msg.header.stamp = current_scan_time;
  ndt_pose_with_covariance_msg.pose.pose = ndt_pose;
  ndt_pose_with_covariance_msg.pose.covariance[0] = 1.0;
  ndt_pose_with_covariance_msg.pose.covariance[7] = 1.0;
  ndt_pose_with_covariance_msg.pose.covariance[14] = 1.0;
  ndt_pose_with_covariance_msg.pose.covariance[21] = 1.0;
  ndt_pose_with_covariance_msg.pose.covariance[28] = 1.0;
  ndt_pose_with_covariance_msg.pose.covariance[35] = 1.0;

  if (convergenced) {
    ndt_pose_publisher_->publish(ndt_pose_msg);
    ndt_pose_with_covariance_publisher_->publish(ndt_pose_with_covariance_msg);
    publishTF(map_frame_id_, "ndt_base_link", ndt_pose_msg);
  }

  std_msgs::msg::Float32 transform_probability;
  transform_probability.data = ndt_->getTransformationProbability();
  transform_probability_publisher_->publish(transform_probability);

  sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
  pcl::toROSMsg(*output_cloud, aligned_cloud_msg);
  aligned_cloud_msg.header = points.header;
  ndt_align_cloud_publisher_->publish(aligned_cloud_msg);
}

void NDTLocalization::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped & initialpose)
{
  RCLCPP_INFO(get_logger(), "initial pose callback.");
  if (initialpose.header.frame_id == map_frame_id_) {
    initial_pose_ = initialpose.pose.pose;
    if (!localization_ready_) localization_ready_ = true;
  } else {
    // TODO transform
    RCLCPP_INFO(
      get_logger(), "frame_id is not same. initialpose.header.frame_id is %s",
      initialpose.header.frame_id.c_str());
  }
}

void NDTLocalization::publishTF(
  const std::string frame_id, const std::string child_frame_id,
  const geometry_msgs::msg::PoseStamped pose)
{
  geometry_msgs::msg::TransformStamped transform_stamped;

  transform_stamped.header.frame_id = frame_id;
  transform_stamped.header.stamp = pose.header.stamp;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform.translation.x = pose.pose.position.x;
  transform_stamped.transform.translation.y = pose.pose.position.y;
  transform_stamped.transform.translation.z = pose.pose.position.z;
  transform_stamped.transform.rotation.w = pose.pose.orientation.w;
  transform_stamped.transform.rotation.x = pose.pose.orientation.x;
  transform_stamped.transform.rotation.y = pose.pose.orientation.y;
  transform_stamped.transform.rotation.z = pose.pose.orientation.z;

  broadcaster_->sendTransform(transform_stamped);
}
