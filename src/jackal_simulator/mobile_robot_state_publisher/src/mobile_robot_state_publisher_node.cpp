/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2018 \n
 *   TU Delft
 *
 *****************************************************************
 *
 * \note
 *   Project name:
 * \note
 *   ROS stack name:
 * \note
 *   ROS package name: mobile_robot_state_publisher
 *
 * \author
 *   Author: Bruno Brito, email: Bruno.deBrito@tudelft.nl
 *
 * \date Date of creation: May, 2018
 *
 * \brief
 *   This package provides a generic mobile_robot_stsate_publisher
 *
 ****************************************************************/

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_broadcaster.h>

using namespace std;

geometry_msgs::Pose pos;
geometry_msgs::Twist vel;
nav_msgs::Odometry odom_msg;

ros::Publisher state_pub_;
ros::Publisher odom_pub_;

ros::Time last_velocity_callback_;
double rate_;

void VelocityCallBack(const gazebo_msgs::LinkStates &msg)
{
	// Update at the given rate
	if (last_velocity_callback_ + ros::Duration(1. / rate_) > ros::Time::now())
		return;

	last_velocity_callback_ = ros::Time::now();

	std::string str2("base_link");

	bool found_index = false;
	size_t index;
	for (index = 0; index < msg.name.size(); index++)
	{

		if (msg.name[index].find(str2) != std::string::npos)
		{
			found_index = true;
			break;
		}
	}

	if (!found_index)
	{
		ROS_WARN_STREAM("Mobile Robot State Publisher: velocity callback() - index not found");
		return;
	}

	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = "odom";
	pose_msg.header.stamp = ros::Time::now();

	// ROS_INFO_STREAM("fOUND IN: " << index);
	pos = msg.pose[index];
	odom_msg.twist.twist = msg.twist[index];
	odom_msg.pose.pose = msg.pose[index];
	odom_msg.child_frame_id = "base_link";
	odom_msg.header.frame_id = "odom";
	odom_msg.header.stamp = ros::Time::now();

	try
	{

		/* This is done because the ukf does not match the position in Gazebo */
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(pos.position.x, pos.position.y, pos.position.z));
		tf::Quaternion q(pos.orientation.x, pos.orientation.y, pos.orientation.z, pos.orientation.w);

		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
	}
	catch (std::runtime_error e)
	{
		std::cout << "mbrobot: exception in transform: " << e.what() << "\n";
	}

	// CONVERT FROM QUATERNION TO JOINT ANGLE ROTATION
	// Intermidiate variables
	double ysqr, t3, t4;
	geometry_msgs::TransformStamped transformStamped;

	ysqr = odom_msg.pose.pose.orientation.y * odom_msg.pose.pose.orientation.y;
	t3 = +2.0 * (odom_msg.pose.pose.orientation.w * odom_msg.pose.pose.orientation.z + odom_msg.pose.pose.orientation.x * odom_msg.pose.pose.orientation.y);
	t4 = +1.0 - 2.0 * (ysqr + odom_msg.pose.pose.orientation.z * odom_msg.pose.pose.orientation.z);

	// Check if the Jackal flipped over (it happens)
	tf::Quaternion q;
	q.setX(odom_msg.pose.pose.orientation.x);
	q.setY(odom_msg.pose.pose.orientation.y);
	q.setZ(odom_msg.pose.pose.orientation.z);
	q.setW(odom_msg.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	if (yaw != yaw) // Check for nan
	{
		yaw = 0.;
		pitch = 0.;
		roll = 0.;
	}

	// Orientation
	pose_msg.pose.orientation.x = roll;
	pose_msg.pose.orientation.y = pitch;
	pose_msg.pose.orientation.z = yaw;

	// Position
	pose_msg.pose.position.x = odom_msg.pose.pose.position.x;
	pose_msg.pose.position.y = odom_msg.pose.pose.position.y;

	// Velocity
	pose_msg.pose.position.z = std::sqrt(std::pow(odom_msg.twist.twist.linear.x, 2) + std::pow(odom_msg.twist.twist.linear.y, 2));

	state_pub_.publish(pose_msg);
	odom_pub_.publish(odom_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mobile_robot_state_publisher_node");
	ros::NodeHandle n;
	ros::Subscriber robot_state_sub_;

	if (!n.getParam(ros::this_node::getName() + "/rate", rate_))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName() + "/rate not set");
		return 0;
	}

	string root_frame;
	if (!n.getParam(ros::this_node::getName() + "/root_frame", root_frame))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName() + "/root_frame not set");
		return 0;
	}

	string base_frame;
	if (!n.getParam(ros::this_node::getName() + "/base_frame", base_frame))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName() + "/base_frame not set");
		return 0;
	}

	string robot_state_topic;
	if (!n.getParam(ros::this_node::getName() + "/robot_state_topic", robot_state_topic))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName() + "/robot_state_topic not set");
		return 0;
	}

	string vel_state_topic;
	if (!n.getParam(ros::this_node::getName() + "/vel_state_topic", vel_state_topic))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName() + "/vel_state_topic not set");
		return 0;
	}

	robot_state_sub_ = n.subscribe(vel_state_topic, 1, VelocityCallBack);

	state_pub_ = n.advertise<geometry_msgs::PoseStamped>(robot_state_topic, 1);
	odom_pub_ = n.advertise<nav_msgs::Odometry>("/odometry/filtered", 1);

	ros::ServiceClient link_state_client_ = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");

	gazebo_msgs::SetLinkState link;
	link.request.link_state.link_name = "base_link";

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);
	last_velocity_callback_ = ros::Time::now();
	ros::spin();

	// ros::Rate rate(node_rate);
	// geometry_msgs::PoseStamped pose_msg;

	// ros::Publisher odom_pub_ = n.advertise<nav_msgs::Odometry>("/odometry/filtered", 1);

	// // Intermidiate variables
	// double ysqr, t3, t4;
	// geometry_msgs::TransformStamped transformStamped;
	// ros::Rate r(rate);
	// while (n.ok())
	// {
	/*
	try{
		transformStamped = tfBuffer.lookupTransform(root_frame, base_frame,
													ros::Time(0));
	}
	catch (tf2::TransformException &ex) {
		ROS_WARN("%s",ex.what());
		ros::Duration(1.0).sleep();
		continue;
	}
	//CONVERT FROM QUATERNION TO JOINT ANGLE ROTATION
	*/
	// pose_msg.header.frame_id = "odom";
	// pose_msg.header.stamp = ros::Time::now();

	// ysqr = odom_msg.pose.pose.orientation.y * odom_msg.pose.pose.orientation.y;
	// t3 = +2.0 * (odom_msg.pose.pose.orientation.w * odom_msg.pose.pose.orientation.z + odom_msg.pose.pose.orientation.x * odom_msg.pose.pose.orientation.y);
	// t4 = +1.0 - 2.0 * (ysqr + odom_msg.pose.pose.orientation.z * odom_msg.pose.pose.orientation.z);

	// // pose_msg.pose.orientation.x = atan2(t3, t4);
	// // pose_msg.pose.orientation.y = atan2(t3, t4);
	// // pose_msg.pose.orientation.z = atan2(t3, t4);
	// // Check if the Jackal flipped over (it happens)
	// tf::Quaternion q;
	// q.setX(odom_msg.pose.pose.orientation.x);
	// q.setY(odom_msg.pose.pose.orientation.y);
	// q.setZ(odom_msg.pose.pose.orientation.z);
	// q.setW(odom_msg.pose.pose.orientation.w);
	// tf::Matrix3x3 m(q);
	// double roll, pitch, yaw;
	// m.getRPY(roll, pitch, yaw);
	// if (yaw != yaw) // Check for nan
	// {
	// 	yaw = 0.;
	// 	pitch = 0.;
	// 	roll = 0.;
	// }

	// // Orientation
	// pose_msg.pose.orientation.x = roll;
	// pose_msg.pose.orientation.y = pitch;
	// pose_msg.pose.orientation.z = yaw;

	// // Position
	// pose_msg.pose.position.x = odom_msg.pose.pose.position.x;
	// pose_msg.pose.position.y = odom_msg.pose.pose.position.y;

	// // Velocity
	// pose_msg.pose.position.z = std::sqrt(std::pow(odom_msg.twist.twist.linear.x, 2) + std::pow(odom_msg.twist.twist.linear.y, 2));

	// state_pub_.publish(pose_msg);
	// /*
	// link.request.link_state.pose.position.x = transformStamped.transform.translation.x;
	// link.request.link_state.pose.position.y = transformStamped.transform.translation.y;
	// link.request.link_state.pose.position.z = transformStamped.transform.translation.z;
	// link.request.link_state.pose.orientation.x = transformStamped.transform.rotation.x;
	// link.request.link_state.pose.orientation.y = transformStamped.transform.rotation.y;
	// link.request.link_state.pose.orientation.z = transformStamped.transform.rotation.z;
	// link.request.link_state.pose.orientation.w = transformStamped.transform.rotation.w;
	// link_state_client_.call(link);
	//  */
	// // link_state_pub_.publish(link);
	// odom_pub_.publish(odom_msg);
	// ros::spinOnce();
	// r.sleep();
	// }

	return 0;
}
