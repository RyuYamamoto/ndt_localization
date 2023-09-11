#include <ndt_localization/ndt_localization.h>

NDTLocalization::NDTLocalization() : Node("ndt_localization")
{
  min_crop_vehicle_x_ = this->declare_parameter<double>("min_crop_vehicle_x");
  max_crop_vehicle_x_ = this->declare_parameter<double>("max_crop_vehicle_x");
  min_crop_vehicle_y_ = this->declare_parameter<double>("min_crop_vehicle_y");
  max_crop_vehicle_y_ = this->declare_parameter<double>("max_crop_vehicle_y");

  min_range_x_ = this->declare_parameter<double>("min_range_x");
  max_range_x_ = this->declare_parameter<double>("max_range_x");
  min_range_y_ = this->declare_parameter<double>("min_range_y");
  max_range_y_ = this->declare_parameter<double>("max_range_y");
  min_range_z_ = this->declare_parameter<double>("min_range_z");
  max_range_z_ = this->declare_parameter<double>("max_range_z");

  correct_translation_offset_ = this->declare_parameter<bool>("correct_translation_offset");
  correct_orientation_offset_ = this->declare_parameter<bool>("correct_orientation_offset");

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

  imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu", 5, std::bind(&NDTLocalization::imu_callback, this, std::placeholders::_1));
  map_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&NDTLocalization::map_callback, this, std::placeholders::_1));
  points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(5),
    std::bind(&NDTLocalization::points_callback, this, std::placeholders::_1));
  initialpose_subscriber_ =
    this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1,
      std::bind(&NDTLocalization::initial_pose_callback, this, std::placeholders::_1));

  ndt_result_path_.header.frame_id = map_frame_id_;

  ndt_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("ndt_result_path", 5);
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
  pcl::PointCloud<PointType>::Ptr output_cloud_ptr)
{
  for (const auto & p : input_cloud_ptr->points) {
    if (
      min_range_x_ < p.x < max_range_x_ and min_range_y_ < p.y < max_range_y_ and
      min_range_z_ < p.z < max_range_z_) {
      if (
        (p.x < min_crop_vehicle_x_ or max_crop_vehicle_x_ < p.x) and
        (p.y < min_crop_vehicle_y_ or max_crop_vehicle_y_ < p.y))
        output_cloud_ptr->points.emplace_back(p);
    }
  }
}

void NDTLocalization::imu_callback(const sensor_msgs::msg::Imu & imu)
{
  imu_queue_.emplace_back(imu);
}

void NDTLocalization::suggest_init_pose_callback(
  const geometry_msgs::msg::PoseStamped & suggest_init_pose)
{
  pose_queue_.emplace_back(suggest_init_pose);
}

void NDTLocalization::map_callback(const sensor_msgs::msg::PointCloud2 & map)
{
  RCLCPP_INFO(get_logger(), "map callback");

  pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(map, *map_cloud);

  ndt_->setInputTarget(map_cloud);
}

void NDTLocalization::points_callback(const sensor_msgs::msg::PointCloud2 & points)
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
  static rclcpp::Time previous_scan_time = current_scan_time;

  const double dt = (current_scan_time - previous_scan_time).seconds();

  pcl::PointCloud<PointType>::Ptr input_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(points, *input_cloud_ptr);

  // downsampling input point cloud
  pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
  downsample(input_cloud_ptr, filtered_cloud);

  // crop point cloud
  pcl::PointCloud<PointType>::Ptr crop_cloud(new pcl::PointCloud<PointType>);
  crop(filtered_cloud, crop_cloud);

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

  // imu fusion
  double roll, pitch, yaw;
  tf2::Quaternion quat(
    initial_pose_.orientation.x, initial_pose_.orientation.y, initial_pose_.orientation.z,
    initial_pose_.orientation.w);
  tf2::Matrix3x3 mat(quat);
  mat.getRPY(roll, pitch, yaw);

  if (!imu_queue_.empty()) {
    // get latest imu data
    sensor_msgs::msg::Imu latest_imu_msgs;
    for (auto & imu : imu_queue_) {
      latest_imu_msgs = imu;
      const auto time_stamp = latest_imu_msgs.header.stamp;
      if (current_scan_time < time_stamp) {
        break;
      }
    }
    while (!imu_queue_.empty()) {
      if (rclcpp::Time(imu_queue_.front().header.stamp) >= current_scan_time) {
        break;
      }
      imu_queue_.pop_front();
    }
    // corrent orientation
    if (correct_orientation_offset_) {
      double roll, pitch, yaw;
      tf2::Quaternion quat(
        initial_pose_.orientation.x, initial_pose_.orientation.y, initial_pose_.orientation.z,
        initial_pose_.orientation.w);
      tf2::Matrix3x3 mat(quat);
      mat.getRPY(roll, pitch, yaw);
      roll += (imu_data_.angular_velocity.x * dt);
      pitch += (imu_data_.angular_velocity.y * dt);
      yaw += (imu_data_.angular_velocity.z * dt);
      quat.setRPY(roll, pitch, yaw);
      initial_pose_.orientation.x = quat.x();
      initial_pose_.orientation.y = quat.y();
      initial_pose_.orientation.z = quat.z();
      initial_pose_.orientation.w = quat.w();
    }

    // correct offset
    if (correct_translation_offset_) {
      double acc_x1 = imu_data_.linear_acceleration.x;
      double acc_y1 = std::cos(roll) * imu_data_.linear_acceleration.y -
                      std::sin(roll) * imu_data_.linear_acceleration.z;
      double acc_z1 = std::sin(roll) * imu_data_.linear_acceleration.y +
                      std::cos(roll) * imu_data_.linear_acceleration.z;

      double acc_x2 = std::sin(pitch) * acc_z1 + std::cos(pitch) * acc_x1;
      double acc_y2 = acc_y1;
      double acc_z2 = std::cos(pitch) * acc_z1 - std::sin(pitch) * acc_x1;

      double acc_x = std::cos(yaw) * acc_x2 - std::sin(yaw) * acc_y2;
      double acc_y = std::sin(yaw) * acc_x2 + std::cos(yaw) * acc_y2;
      double acc_z = acc_z2;

      double offset_translation_imu_x = imu_velocity_.x * dt + acc_x * dt * dt / 2.0;
      double offset_translation_imu_y = imu_velocity_.y * dt + acc_y * dt * dt / 2.0;
      double offset_translation_imu_z = imu_velocity_.z * dt + acc_z * dt * dt / 2.0;

      imu_velocity_.x += acc_x * dt;
      imu_velocity_.y += acc_y * dt;
      imu_velocity_.z += acc_z * dt;

      initial_pose_.position.x += offset_translation_imu_x;
      initial_pose_.position.y += offset_translation_imu_y;
      initial_pose_.position.z += offset_translation_imu_z;
    }
  }

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

  // estimate velocity
  double dx = ndt_pose.position.x - initial_pose_.position.x;
  double dy = ndt_pose.position.y - initial_pose_.position.y;
  double dz = ndt_pose.position.z - initial_pose_.position.z;
  velocity_.x = (dt > 0.0) ? (dx / dt) : 0.0;
  velocity_.y = (dt > 0.0) ? (dy / dt) : 0.0;
  velocity_.z = (dt > 0.0) ? (dz / dt) : 0.0;
  imu_velocity_ = velocity_;
  initial_pose_ = ndt_pose;

  geometry_msgs::msg::PoseStamped ndt_pose_msg;
  ndt_pose_msg.header.frame_id = map_frame_id_;
  ndt_pose_msg.header.stamp = current_scan_time;
  ndt_pose_msg.pose = ndt_pose;

  ndt_result_path_.poses.emplace_back(ndt_pose_msg);

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
    publish_tf(map_frame_id_, "base_link", ndt_pose_msg);
  }

  ndt_result_path_.header.stamp = current_scan_time;
  ndt_path_publisher_->publish(ndt_result_path_);

  std_msgs::msg::Float32 transform_probability;
  transform_probability.data = ndt_->getTransformationProbability();
  transform_probability_publisher_->publish(transform_probability);

  sensor_msgs::msg::PointCloud2 aligned_cloud_msg;
  pcl::toROSMsg(*output_cloud, aligned_cloud_msg);
  aligned_cloud_msg.header = points.header;
  ndt_align_cloud_publisher_->publish(aligned_cloud_msg);

  previous_scan_time = current_scan_time;
}

void NDTLocalization::initial_pose_callback(
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
  imu_velocity_.x = 0.0;
  imu_velocity_.y = 0.0;
  imu_velocity_.z = 0.0;
  velocity_.x = 0.0;
  velocity_.y = 0.0;
  velocity_.z = 0.0;
}

void NDTLocalization::publish_tf(
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
