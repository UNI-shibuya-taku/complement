#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include "complement/odom_publisher.h"

using std::cout;
using std::endl;
using std::string;

namespace complement
{
	template<typename WheelT, typename GyroT>
	odomPublisher<WheelT, GyroT>::odomPublisher()
		: sync(SyncPolicy(150), sub_wheel, sub_gyro),
		  n("~"), x(0.0), y(0.0), pitch(0.0), yaw(0.0), vel(0.0), dyaw(0.0), isfirst(true), rate(1.0)
	{
		n.param<string>("topic_name/wheel", topic_wheel, "/odom");
		n.param<string>("topic_name/gyro", topic_gyro, "/imu/data");
		n.param<string>("topic_name/odom_complement", topic_pub, "/odom/complement");

		n.param<string>("frame_name/this", frame_this, "/odom");
		n.param<string>("frame_name/child", frame_child, "/base_link");

		sub_wheel.subscribe(n, topic_wheel, 10);
		sub_gyro.subscribe(n, topic_gyro, 150);

		sync.registerCallback(boost::bind(&odomPublisher::callback, this, _1, _2));

		pub_odom = n.advertise<nav_msgs::Odometry>(topic_pub, 1);

		if(!n.getParam("dyaw/drift", drift_dyaw)){
			drift_dyaw = 0.0;
		}
		cout << "drift_dyaw: " << drift_dyaw << endl;
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::callback
	(const typename WheelT::ConstPtr& wheel, const typename GyroT::ConstPtr& gyro)
	{
		wheelCallback(wheel);
		gyroCallback(gyro);
		complement();
		publisher();
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::wheelCallback(const typename WheelT::ConstPtr& msg)
	{
		current_time = msg->header.stamp;
		if(isfirst){
			last_time = current_time;
			isfirst = false;
		}
		vel = msg->twist.twist.linear.x;
		vel = vel > 1.5 ? 1.5 : vel;

		vel *= rate;
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::gyroCallback(const sensor_msgs::Imu::ConstPtr& msg)
	{
		dyaw = msg->angular_velocity.z - drift_dyaw;

		dyaw *= rate;
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::complement()
	{
		double dt = (current_time - last_time).toSec();
		last_time = current_time;

		if(0.1 < dt) return;

		double dist = vel * dt;

		yaw += dyaw * dt;

		x += dist * cos(yaw);
		y += dist * sin(yaw);
		odom_quat = tf::createQuaternionMsgFromYaw(yaw);

		vel = dyaw = 0.0;
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::publisher()
	{
		nav_msgs::Odometry odom;
		odom.header.frame_id = frame_this;
		odom.child_frame_id = frame_child;
		odom.header.stamp = current_time;

		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		pub_odom.publish(odom);
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::pubTF()
	{
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = frame_this;
		odom_trans.child_frame_id = frame_child;

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		odom_broadcaster.sendTransform(odom_trans);
	}

	template<typename WheelT, typename GyroT>
	void odomPublisher<WheelT, GyroT>::setRate(const double& bag_rate)
	{
		rate = bag_rate;
	}
}

