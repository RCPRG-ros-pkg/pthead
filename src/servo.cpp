#include "ros/ros.h"
#include <stdio.h>
#include <cstring>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"

#include <time.h>
#include <unistd.h>
#include <assert.h>

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"

#define COEFF_P			4.0
#define ANGLE_INSENS	0.25
#define INCREM_LIMIT	70.0

std::string nodeName = "ptservo";

//bool trackerPaused	= false;
//bool trackerLog	= false;
//bool pthSynchronize	= false;


/**
* 
*/
float insensitive(float v, float lvl) {
	lvl = fabs(lvl)
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
void posCallback(const geometry_msgs::PointStamped& msg) {
	float xx, yy;
	xx = msg->point.x;
	yy = msg->point.y;
	
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
}

/**
* 
*/
int main(int argc, char **argv)
{
	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	ros::Subscriber pos_sub = n.subscribe("pos_error", 1, &posCallback);
	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("head_vel", 10);

	ROS_INFO("Starting node %s", nodeName.c_str());
	
	ros::spin();
	return 0;
}
