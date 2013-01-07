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

#define COEFF_P			250.0
#define ANGLE_INSENS	0.05
#define INCREM_LIMIT	80.0

std::string nodeName = "ptservo";

FILE * logfile;

bool trackerPaused	= false;
bool trackerLog	= false;
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

	if(msg.buttons[PS3_BUTTON_ACTION_SQUARE] == 1
			&& prevmsg.buttons[PS3_BUTTON_ACTION_SQUARE] == 0){
		if(trackerLog) {
			trackerLog = false;
			ROS_INFO("Logging OFF");
		}
		else {
			trackerLog = true;
			ROS_INFO("Logging ON");
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
	xx = inrange(xx, -INCREM_LIMIT, INCREM_LIMIT);
	yy = inrange(yy, -INCREM_LIMIT, INCREM_LIMIT);
	
	geometry_msgs::Twist	msgTwist;
	msgTwist.angular.z = xx;
	msgTwist.angular.y = yy;
	twist_pub.publish(msgTwist);
	
	//if(trackerLog)
	//    fprintf (logfile, "rotx %5f, roty %5f \n", msg.point.x, msg.point.y);
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
	
	//logfile = fopen ("pstrackerlog.txt","a");

	ROS_INFO("Starting node %s", nodeName.c_str());
	
	ros::spin();
	return 0;
}
