#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <geometry_msgs/TwistStamped.h>
#include "ptproxy/PTProxy.h"
#include <stdio.h>
#include <cstring>

#include "sensor_msgs/Joy.h"
#include "psdefs.h"

#define MOVE_TO_CENTER_SPEED	1.0 //[rad/s]

std::string nodeName = "pthead";
PTProxy *ptp;
float dx = 0, dy = 0, rx, ry;

ros::Subscriber joy_sub;

/**
* 
*/
void twistCallback(const geometry_msgs::TwistStamped& msg) {
	dx = msg.twist.angular.z;
	dy = msg.twist.angular.y;
	ptp->setJointSpeed(dx, dy);
}

/**
* 
*/
void joyCallback(const sensor_msgs::Joy& msg) {
	static sensor_msgs::Joy prevmsg;

	if(msg.buttons[PS3_BUTTON_ACTION_CROSS] == 1
			&& prevmsg.buttons[PS3_BUTTON_ACTION_CROSS] == 0){
		ptp->setJointPositionWithSpeed(0, 0, MOVE_TO_CENTER_SPEED, MOVE_TO_CENTER_SPEED);	
	}
	
	prevmsg = msg;
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
	joy_sub = n.subscribe("psmove_out", 1, &joyCallback);
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
	
	int spinCnt = 1;
	while (ros::ok())
	{
		// Read - write hardware
		ptp->nextStep();
		
		if(spinCnt < 3) {
			// Do nothing, only request motor drivers' status
			spinCnt ++;
		}
		else if(spinCnt == 3) {
			if(ptp->isSynchronized() == 0)
			// If head is not synchronized, prepare synchronization request
			ptp->startSynchronization();
			spinCnt ++;
		}
		else {
			// If head is synchronized
			if(ptp->isSynchronized()){
				// Process received data
				ptp->getJointPosition(rx, ry);
				// Building message
				msg.header.stamp = ros::Time::now();
				msg.position[0] = rx;
				msg.position[1] = ry;
				js_pub.publish(msg);
			}
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
