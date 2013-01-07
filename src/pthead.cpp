#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/Twist.h>
#include "ptproxy/PTProxy.h"
#include <stdio.h>
#include <cstring>

std::string nodeName = "pthead";
PTProxy *ptp;
float dx = 0, dy = 0, rx, ry;

/**
* 
*/
void twistCallback(const geometry_msgs::TwistConstPtr& msg) {
	dx = msg->angular.z;
	dy = msg->angular.y;
	ptp->setMotorSpeed(dx, dy);
}

/**
* 
*/
int main(int argc, char **argv)
{
  sensor_msgs::JointState msg;

	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	ros::Subscriber twist_sub = n.subscribe("head_vel", 1, &twistCallback);
	ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("head_jstates", 10);
	ros::Rate loop_rate(100);

	msg.name.resize(2);
	msg.name[0] = "head_pan_joint";
	msg.name[1] = "head_tilt_joint";
	
	msg.position.resize(2);
	msg.position[0] = 0.0;
	msg.position[1] = 0.0;
  
	msg.velocity.resize(2);
	msg.effort.resize(2);

	ROS_INFO("Starting node %s", nodeName.c_str());
	
	ptp = new PTProxy("/dev/ttyACM0");
	// Prepare synchronization mode request
	ptp->startSynchronization();
	// Send it to motor controllers
	ptp->nextStep();
	
	bool quit = false;
	while (ros::ok())
	{
		// Read - write hardware
		ptp->nextStep();
		// Process received data
		ptp->getJointPosition(rx, ry);
		
		// Building message
		msg.header.stamp = ros::Time::now();
	    msg.position[0] = rx;
	    msg.position[1] = ry;
		js_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
