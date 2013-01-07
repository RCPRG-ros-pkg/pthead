#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include "ptproxy/PTProxy.h"
#include <stdio.h>
#include <cmath>
#include <cstring>

std::string nodeName = "iptsc";

ros::Subscriber pt_sub;
ros::Publisher pt_pub;

/**
 *
 */
void ptCallback(const geometry_msgs::PointStampedPtr& msg) {
	geometry_msgs::PointStamped ret;
	sensor_msgs::CameraInfo ci;

	// przechera!
	ci.width = 640;
	ci.height = 480;
	ci.K[0] = 548;
	ci.K[4] = 548;
	ci.K[2] = 319.5;
	ci.K[5] = 239.5;
	ci.K[8] = 1;

	float ex, ey;
	ex = msg->point.x - ci.K[2];
	ey = msg->point.y - ci.K[5];

	ret.header = msg->header;
	ret.point.x = atan2(ex, ci.K[0]);
	ret.point.y = atan2(ey, ci.K[4]);

	pt_pub.publish(ret);
}

/**
 *
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	pt_sub = n.subscribe("image_position", 1, &ptCallback);
	pt_pub = n.advertise <geometry_msgs::PointStamped> ("spherical_position", 10);

	ROS_INFO("Starting node %s", nodeName.c_str());

	ros::spin();

	return 0;
}
