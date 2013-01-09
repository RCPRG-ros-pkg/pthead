#include "ros/ros.h"
#include <stdio.h>
#include <cstring>
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"

#include <time.h>
#include <unistd.h>
#include <assert.h>

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"

#include "psdefs.h"

#define COEFF_P				0.14
#define ANGLE_INSENS		0.1
#define ANG_INCREM_LIMIT	100

std::string nodeName = "ptservo";

FILE * logfile;

bool trackerPaused	= false;
//bool pthSynchronize	= false;

ros::Subscriber pos_sub;
ros::Subscriber joy_sub;
ros::Subscriber joints_sub;
ros::Publisher twist_pub;

/**
* 
*/
float insensitive(float v, float lvl) {
	lvl = fabs(lvl);
	return (v > lvl ? (v - lvl) : (v < (-lvl) ? (v + lvl) : 0) );
}

/**
* 
*/
float inrange(float v, float mn, float mx) {
	return (v < mn ? mn : (v > mx ? mx : v) );
}

/**
* 
*/
void joyCallback(const sensor_msgs::Joy& msg) {
	static sensor_msgs::Joy prevmsg;

	if(msg.buttons[PS3_BUTTON_REAR_RIGHT_1] == 1
			&& prevmsg.buttons[PS3_BUTTON_REAR_RIGHT_1] == 0){
		if(trackerPaused) {
			trackerPaused = false;
			ROS_INFO("PSMove Tracker RESUMED");
		}
		else {
			
			trackerPaused = true;
			geometry_msgs::Twist	msgTwist;
			msgTwist.angular.z = 0;
			msgTwist.angular.y = 0;
			twist_pub.publish(msgTwist);
			ROS_INFO("PSMove Tracker PAUSED");
		}
	}
	
	prevmsg = msg;
}

/**
* 
*/
void posCallback(const geometry_msgs::PointStamped& msg) {
	if(trackerPaused)
		return;
	float xx, yy;
	xx = msg.point.x;
	yy = msg.point.y;
	
	xx = insensitive(xx, ANGLE_INSENS);
	yy = insensitive(yy, ANGLE_INSENS);
	xx *= COEFF_P;
	yy *= COEFF_P;
	xx = inrange(xx, -ANG_INCREM_LIMIT, ANG_INCREM_LIMIT);
	yy = inrange(yy, -ANG_INCREM_LIMIT, ANG_INCREM_LIMIT);
	
	geometry_msgs::Twist	msgTwist;
	msgTwist.angular.z = xx;
	msgTwist.angular.y = yy;
	twist_pub.publish(msgTwist);
}

/**
* 
*/
void jstatesCallback(const sensor_msgs::JointState& msg) {

}

/**
* 
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	pos_sub = n.subscribe("pos_error", 1, &posCallback);
	joints_sub = n.subscribe("head_jstates", 1, &jstatesCallback);
	joy_sub = n.subscribe("psmove_out", 1, &joyCallback);
	twist_pub = n.advertise<geometry_msgs::Twist>("head_vel", 10);

	ROS_INFO("Starting node %s", nodeName.c_str());
	
	ros::spin();
	return 0;
}
