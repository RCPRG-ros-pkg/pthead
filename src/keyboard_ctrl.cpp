#include "ros/ros.h"
#include <stdio.h>
#include <cstring>
#include <termios.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"

std::string nodeName = "ptkeybctrl";

/**
* 
*/
int main(int argc, char **argv)
{
	sensor_msgs::JointState	msgJointState;
	geometry_msgs::Twist	msgTwist;

	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	ros::Publisher js_pub = n.advertise<geometry_msgs::Twist>("/head_vel", 10);
	ros::Rate loop_rate(10);

	msgJointState.name.resize(2);
	msgJointState.name[0] = "head_pan_joint";
	msgJointState.name[1] = "head_tilt_joint";
	msgJointState.position.resize(2);
	msgJointState.position[0] = 0.0;
	msgJointState.position[1] = 0.0;
	msgJointState.velocity.resize(2);
	msgJointState.effort.resize(2);

	ROS_INFO("Starting node %s", nodeName.c_str());
	
	char inp;
	float dx = 0, dy = 0;
	bool synchronized = false;
	// get the console in raw mode
	int kfd = 0;
	struct termios cooked, raw;
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~(ICANON);
	raw.c_lflag &= ~(ECHO);
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	raw.c_cc[VMIN] = 0;
	raw.c_cc[VTIME] = 0;
	tcsetattr(kfd, TCSANOW, &raw);
	// read() is non-blocking from now on and echo is off
	
	printf("Pan-tilt speed control: [A] x- [D] x+ [S] y- [W] y+ [space] stop\n\n");
	printf("Quit: [Q]\n\n");
	sleep(1);
	
	bool quit = false;
	while (ros::ok() && (!quit))
	{	// Prepare data to send
		if(read(0, (void*)&inp, 1)){
			switch(inp){
				case 'a':
				case 'A':
					dx -= 1;
					break;
				case 'd':
				case 'D':
					dx += 1;
					break;
				case 's':
				case 'S':
					dy -= 1;
					break;
				case 'w':
				case 'W':
					dy += 1;
					break;
				case ' ':
					dx = 0;
					dy = 0;
					break;
				case 'q':
				case 'Q':
					dx = 0;
					dy = 0;
					quit = true;
					break;
			}
		}
		
		// Building message
		msgTwist.angular.z = dx;
		msgTwist.angular.y = dy;
		js_pub.publish(msgTwist);

		ros::spinOnce();

		loop_rate.sleep();
	}

	// restore console settings
	raw.c_lflag |= ECHO | ICANON;
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	raw.c_cc[VMIN] = 0;
	raw.c_cc[VTIME] = 0;
	tcsetattr(kfd, TCSANOW, &raw);

	return 0;
}
