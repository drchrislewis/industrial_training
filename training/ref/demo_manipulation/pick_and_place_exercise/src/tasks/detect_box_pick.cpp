/*
 * detect_box_pick.cpp
 *
 *  Created on: Jun 3, 2013
 *      Author: ros-industrial
 */

#include <pick_and_place_exercise/pick_and_place.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/transforms.h>

/* DETECTING BOX PICK POSE
  Goal:
    - Find the box's position in the world frame using the transform listener.
        * this transform is published by the kinect AR-tag perception node
    - Save the pose into 'box_pose'.

  Hints:
    - lookupTransform can also look "in the past".  Use Time=0 to get the most-recent transform.
    - tf::poseTFToMsg allows converting transforms into Pose messages
*/
geometry_msgs::Pose detect_box_pick(tf::TransformListener &tf_listener)
{
  //ROS_ERROR_STREAM("detect_box_pick is not implemented yet.  Aborting."); exit(1);

  // task variables
  tf::StampedTransform world_to_box_pick_tf;
  geometry_msgs::Pose box_pose;

  // use transform listener to find the box's pick pose (relative to world frame)
  /* Fill Code: [ use the 'lookupTransform' method in the transform listener] */
  tf_listener.lookupTransform(cfg.WORLD_FRAME_ID,cfg.BOX_PICK_FRAME_ID,ros::Time(0.0f),world_to_box_pick_tf);

  // save pose in 'box_pose'
  /* Fill Code: [ use the 'tf::poseTFToMsg' to convert a TF transform into a pose message] */
  tf::poseTFToMsg(world_to_box_pick_tf,box_pose);

/*  // detecting height
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  sensor_msgs::PointCloud2ConstPtr sensor_cloud =
		  ros::topic::waitForMessage<sensor_msgs::PointCloud2>(cfg.POINT_CLOUD_TOPIC,ros::Duration(2.0f));

  if(sensor_cloud == sensor_msgs::PointCloud2ConstPtr())
  {
	  // no point cloud message received, defaulting to box height
	  box_pose.position.z = cfg.BOX_SIZE.getZ();
	  ROS_INFO_STREAM("No point cloud message received, using box height "<<cfg.BOX_SIZE.getZ());
  }
  else
  {
	  ROS_INFO_STREAM("point cloud message received, estimating surface");

	  // converting message to point cloud type
	  pcl::fromROSMsg<pcl::PointXYZ>(*sensor_cloud,*raw_cloud);

	  // applying transform
	  pcl_ros::transformPointCloud<pcl::PointXYZ>(cfg.WORLD_FRAME_ID,
			  *raw_cloud,*cloud,tf_listener);

	  // applying filter in x axis
	  float min = box_pose.position.x - 0.2*cfg.BOX_SIZE.x();
	  float max = box_pose.position.x + 0.2*cfg.BOX_SIZE.x();
	  pcl::PassThrough<pcl::PointXYZ> filter;
	  filter.setInputCloud(cloud);
	  filter.setFilterFieldName("x");
	  filter.setFilterLimits(min,max);
	  filter.filter(*cloud);

	  // applying filter in y axis
	  min = box_pose.position.y - 0.2*cfg.BOX_SIZE.y();
	  max = box_pose.position.y + 0.2*cfg.BOX_SIZE.y();
	  filter.setInputCloud(cloud);
	  filter.setFilterFieldName("y");
	  filter.setFilterLimits(min,max);
	  filter.filter(*cloud);

	  // computing centroid
	  Eigen::Vector4f centroid;
	  pcl::compute3DCentroid(*cloud,centroid);

	  // setting z value
	  box_pose.position.z = centroid[2];
  }

  set_object_in_world(true,box_pose);*/

  return box_pose;
}

