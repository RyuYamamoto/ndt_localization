#include <ndt_localization/ndt_localization.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>

NDTLocalization::NDTLocalization()
{
  pnh_.param<double>("transformation_epsilon", transformation_epsilon_, 0.01);
  pnh_.param<double>("step_size", step_size_, 0.1);
  pnh_.param<double>("ndt_resolution", ndt_resolution_, 5.0);
  pnh_.param<int>("max_iteration", max_iteration_, 20);
  pnh_.param<int>("omp_num_thread", omp_num_thread_, 3);
  pnh_.param<std::string>("map_frame_id", map_frame_id_, "map");
  pnh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");

  ndt_.reset(new pclomp::NormalDistributionsTransform<PointType, PointType>);

  ndt_->setTransformationEpsilon(transformation_epsilon_);
  ndt_->setStepSize(step_size_);
  ndt_->setResolution(ndt_resolution_);
  ndt_->setMaximumIterations(max_iteration_);
  ndt_->setNeighborhoodSearchMethod(pclomp::KDTREE);
  if (0 < omp_num_thread_) ndt_->setNumThreads(omp_num_thread_);

  map_subscriber_ = pnh_.subscribe("points_map", 1, &NDTLocalization::mapCallback, this);
  points_subscriber_ = pnh_.subscribe("points_raw", 1, &NDTLocalization::pointsCallback, this);
  initialpose_subscriber_ =
    pnh_.subscribe("/initialpose", 1, &NDTLocalization::initialPoseCallback, this);

  ndt_align_cloud_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("aligned_cloud", 1);
  ndt_pose_publisher_ = pnh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 1);
  transform_probability_publisher_ = pnh_.advertise<std_msgs::Float32>("transform_probability", 1);
}

void NDTLocalization::mapCallback(const sensor_msgs::PointCloud2 & map)
{
  ROS_INFO("map callback");

  pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(map, *map_cloud);

  ndt_->setInputTarget(map_cloud);
}

void NDTLocalization::pointsCallback(const sensor_msgs::PointCloud2 & points)
{
  if (ndt_->getInputTarget() == nullptr) {
    ROS_ERROR("map not received!");
    return;
  }

	if(!localization_ready_) {
		ROS_ERROR("initial pose not received!");
		return;
	}

  const ros::Time current_scan_time = points.header.stamp;

	pcl::PointCloud<PointType>::Ptr input_cloud_ptr(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr filtered_cloud(new pcl::PointCloud<PointType>);
	pcl::fromROSMsg(points, *input_cloud_ptr);
	pcl::VoxelGrid<PointType> voxel_grid;
	voxel_grid.setLeafSize(3.0, 3.0, 3.0);
	voxel_grid.setInputCloud(input_cloud_ptr);
	voxel_grid.filter(*filtered_cloud);
	ndt_->setInputSource(filtered_cloud);

  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();

  Eigen::Affine3d initial_pose_affine;
  tf2::fromMsg(initial_pose_, initial_pose_affine);
  init_guess = initial_pose_affine.matrix().cast<float>();

  pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  ndt_->align(*output_cloud, init_guess);

  const bool convergenced = ndt_->hasConverged();
  const int final_iterations = ndt_->getFinalNumIteration();

  const Eigen::Matrix4f result_ndt_pose = ndt_->getFinalTransformation();

  Eigen::Affine3d result_ndt_pose_affine;
  result_ndt_pose_affine.matrix() = result_ndt_pose.cast<double>();
  const geometry_msgs::Pose ndt_pose = tf2::toMsg(result_ndt_pose_affine);
	initial_pose_ = ndt_pose;

  geometry_msgs::PoseStamped ndt_pose_msg;
  ndt_pose_msg.header.frame_id = map_frame_id_;
  ndt_pose_msg.header.stamp = current_scan_time;
	ndt_pose_msg.pose = ndt_pose;

	if(convergenced) {
		ndt_pose_publisher_.publish(ndt_pose_msg);
		publishTF(map_frame_id_, base_frame_id_, ndt_pose_msg);
	}

	std_msgs::Float32 transform_probability;
	transform_probability.data = ndt_->getTransformationProbability();
	transform_probability_publisher_.publish(transform_probability);

  sensor_msgs::PointCloud2 aligned_cloud_msg;
  pcl::toROSMsg(*output_cloud, aligned_cloud_msg);
  aligned_cloud_msg.header = points.header;
  ndt_align_cloud_publisher_.publish(aligned_cloud_msg);
}

void NDTLocalization::initialPoseCallback(
  const geometry_msgs::PoseWithCovarianceStamped & initialpose)
{
  ROS_INFO("initial pose callback.");
  if (initialpose.header.frame_id == map_frame_id_) {
    initial_pose_ = initialpose.pose.pose;
		if(!localization_ready_) localization_ready_ = true;
  } else {
    // TODO transform
    ROS_ERROR(
      "frame_id is not same. initialpose.header.frame_id is %s",
      initialpose.header.frame_id.c_str());
  }
}

void NDTLocalization::publishTF(const std::string frame_id, const std::string child_frame_id, const geometry_msgs::PoseStamped pose)
{
	geometry_msgs::TransformStamped transform_stamped;

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

	broadcaster_.sendTransform(transform_stamped);
}