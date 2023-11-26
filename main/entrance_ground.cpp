//
// Created by ghm on 2021/12/9.
//

#include "entrance_ground.h"

namespace cvr_lse {
void EntranceGround::Init() {
	double delay_time;
	nh_.param<double>("/ground_segmentation/delay_time", delay_time, 1.5);
	delay_timer_ = nh_.createTimer(ros::Duration(delay_time), &EntranceGround::OnTimer, this, true);
}

void EntranceGround::OnTimer(const ros::TimerEvent&) {
	if (!this->Start()) {
		ROS_ERROR("This node did not initialize properly!!!");
	}
	else {
		ROS_INFO("Node started properly.");
	}
}

bool EntranceGround::Start() {
	// for segmentation
	if (!LoadParameterForGroundSegmentation()) {
		ROS_ERROR("params load error!");
		return false;
	}

	ground_seg_ = std::make_shared<GroundSegmentation>(ground_segmentation_params_);

	std::string cloud_label_topic;
	nh_.param<std::string>("/ground_segmentation/cloud_label_topic", cloud_label_topic,
		"/ground_segmentation/cloud_label");
	ground_label_ =  nh_.advertise<cvr_lse::cloud_label>(cloud_label_topic, 1);

	std::string lidar_topic;
	nh_.param<std::string>("/ground_segmentation/lidar_topic", lidar_topic, "/hesai40p_points_xyzirt");
	point_cloud_sub_ = nh_.subscribe(lidar_topic, 2, &cvr_lse::EntranceGround::PointCloudCallback,
		this);

	return true;
}

bool EntranceGround::LoadParameterForGroundSegmentation() {
	nh_.param<int>("ground_segmentation/n_bins", ground_segmentation_params_.n_bins,
	               ground_segmentation_params_.n_bins);
	nh_.param<int>("ground_segmentation/n_segments", ground_segmentation_params_.n_segments,
	               ground_segmentation_params_.n_segments);
	nh_.param<double>("ground_segmentation/max_dist_to_line", ground_segmentation_params_.max_dist_to_line,
	                  ground_segmentation_params_.max_dist_to_line);
	nh_.param<double>("ground_segmentation/max_slope", ground_segmentation_params_.max_slope,
	                  ground_segmentation_params_.max_slope);
	nh_.param<double>("ground_segmentation/long_threshold", ground_segmentation_params_.long_threshold,
	                  ground_segmentation_params_.long_threshold);
	nh_.param<double>("ground_segmentation/max_long_height", ground_segmentation_params_.max_long_height,
	                  ground_segmentation_params_.max_long_height);
	nh_.param<double>("ground_segmentation/max_start_height", ground_segmentation_params_.max_start_height,
	                  ground_segmentation_params_.max_start_height);
	nh_.param<double>("ground_segmentation/sensor_height", ground_segmentation_params_.sensor_height,
	                  ground_segmentation_params_.sensor_height);
	nh_.param<double>("ground_segmentation/line_search_angle", ground_segmentation_params_.line_search_angle,
	                  ground_segmentation_params_.line_search_angle);
	nh_.param<int>("ground_segmentation/n_threads", ground_segmentation_params_.n_threads,
	               ground_segmentation_params_.n_threads);
	// Params that need to be squared.
	double r_min, r_max, max_fit_error;
	if (nh_.getParam("ground_segmentation/r_min", r_min)) {
		ground_segmentation_params_.r_min_square = r_min * r_min;
	}
	if (nh_.getParam("ground_segmentation/r_max", r_max)) {
		ground_segmentation_params_.r_max_square = r_max * r_max;
	}
	if (nh_.getParam("ground_segmentation/max_fit_error", max_fit_error)) {
		ground_segmentation_params_.max_error_square = max_fit_error * max_fit_error;
	}
	return true;
}

void EntranceGround::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg) {
	cvr_lse::cloud_label::Ptr label_ptr = boost::make_shared<cvr_lse::cloud_label>();
	label_ptr->header = cloudMsg->header;
	pcl::PointCloud<PointXYZIRT> cloud, cloud_tf;
	pcl::fromROSMsg(*cloudMsg, cloud);
	geometry_msgs::Quaternion qt_to_odom;
	try {
		qt_to_odom = tf_buffer_.lookupTransform("odom", cloudMsg->header.frame_id, cloudMsg->header.stamp).transform.rotation;
		found_tf_ = true;
	}
	catch (tf2::TransformException &ex) {
		if (found_tf_)
			ROS_ERROR("%s",ex.what());
		return;
	}
	Eigen::Affine3f pc_transform = Eigen::Affine3f::Identity();
	pc_transform.rotate(Eigen::Quaternionf(qt_to_odom.w, qt_to_odom.x, qt_to_odom.y, qt_to_odom.z));
	pcl::transformPointCloud(cloud, cloud_tf, pc_transform);
	ground_seg_->SegmentProcess(cloud_tf, label_ptr);
	ground_label_.publish(*label_ptr);
	ground_seg_->Reset();
}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "ground");
	ros::NodeHandle nh("~");
	cvr_lse::EntranceGround enter_node(nh);
	enter_node.Init();
	ros::spin();
}