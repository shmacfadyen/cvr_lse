//
// Created by ghm on 2021/12/9.
//

#ifndef CVR_LSE_ENTRANCE_GROUND_H
#define CVR_LSE_ENTRANCE_GROUND_H
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <deque>
#include <string>

//
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "ground_segmentation.h"
namespace cvr_lse {
using namespace ground_segmentation;
class EntranceGround {
public:
	explicit EntranceGround(const ros::NodeHandle &nh) : nh_(nh), listener_(tf_buffer_), found_tf_(false) {};

	~EntranceGround() = default;

	void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

	void Init();
	
	void OnTimer(const ros::TimerEvent&);
	
	bool Start();

	EntranceGround(const EntranceGround &) = delete;

	EntranceGround &operator=(const EntranceGround &) = delete;

	EntranceGround(EntranceGround &&) = delete;

	EntranceGround &operator=(EntranceGround &&) = delete;

private:

	bool LoadParameterForGroundSegmentation();

	std::shared_ptr<GroundSegmentation> ground_seg_;

	GroundSegmentationParams ground_segmentation_params_;

	ros::NodeHandle nh_;

	ros::Publisher ground_label_;
	ros::Subscriber point_cloud_sub_;

	ros::Timer delay_timer_;

	tf2_ros::TransformListener listener_;
	tf2_ros::Buffer tf_buffer_;
	bool found_tf_;
};
}

#endif //CVR_LSE_ENTRANCE_GROUND_H
