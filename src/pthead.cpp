#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "ptproxy/PTProxy.h"
#include <stdio.h>
#include <cstring>

std::string nodeName = "pthead";

/**
* 
*/
int main(int argc, char **argv)
{
  sensor_msgs::JointState msg;

	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
	ros::Rate loop_rate(10);

  msg.name.resize(2);
  msg.name[0] = "head_pan_joint";
  msg.name[1] = "head_tilt_joint";
	
	msg.position.resize(2);
  msg.position[0] = 0.0;
  msg.position[1] = 0.0;
  
  msg.velocity.resize(2);
  msg.effort.resize(2);

	ROS_INFO("Starting node %s", nodeName.c_str());
	
	// get the console in raw mode
	char inp;
	float dx = 0, dy = 0, rx, ry;
	bool synchronized = false;
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
	
	PTProxy *ptp = new PTProxy("/dev/ttyACM0");
	// Prepare synchronization mode request
	ptp->startSynchronization();
	// Send it to motor controllers
	ptp->nextStep();
	
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
					ptp->setMotorSpeed(dx, dy);
					break;
				case 'd':
				case 'D':
					dx += 1;
					ptp->setMotorSpeed(dx, dy);
					break;
				case 's':
				case 'S':
					dy -= 1;
					ptp->setMotorSpeed(dx, dy);
					break;
				case 'w':
				case 'W':
					dy += 1;
					ptp->setMotorSpeed(dx, dy);
					break;
				case ' ':
					dx = 0;
					dy = 0;
					ptp->setMotorSpeed(dx, dy);
					break;
				case 'q':
				case 'Q':
					ptp->setMotorSpeed(0, 0);
					quit = true;
					break;
			}
		}
		// Read - write hardware
		ptp->nextStep();
		// Process received data
		ptp->getJointPosition(rx, ry);
		//printf("px: %6d, py: %6d, syn: %d\n", (int)rx, (int)ry, ptp->isSynchronized());
		
		// Building message
		msg.header.stamp = ros::Time::now();
	    msg.position[0] = rx;
	    msg.position[1] = ry;
		js_pub.publish(msg);

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
