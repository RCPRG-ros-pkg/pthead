#include "ros/ros.h"
#include "ptproxy/PTProxy.h"
#include <geometry_msgs/TwistStamped.h>
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/QueryTrajectoryState.h>
#include "velocityprofile_spline.hpp"
#include <stdio.h>
#include <cstring>
//
#include "sensor_msgs/Joy.h"
#include "psdefs.h"

#define MOVE_TO_CENTER_SPEED	1.0 //[rad/s]
#define SPIN_FREQ				100

std::string nodeName = "pthead";
PTProxy *ptp;
float dx = 0, dy = 0;

std::vector<VelocityProfile_Spline> vel_profile_;
trajectory_msgs::JointTrajectoryPoint trajectory_current_;
trajectory_msgs::JointTrajectoryPoint trajectory_next_;
std::vector<double> des_jnt_pos_;
unsigned int number_of_joints_ = 2;
bool trajectory_ready_;
bool buffer_ready_;
int64_t time_;
int64_t end_time_;
double dt_;
std::vector<std::string> joint_names_;
std::vector<double> q, qd, qdd;
std::vector<double> meas_q, meas_qd, prev_meas_q;

sensor_msgs::JointState joint_state_msg;
pr2_controllers_msgs::JointTrajectoryControllerState trajectory_state_msg;

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
void trajectoryCallback(const trajectory_msgs::JointTrajectory& msg) {
	ROS_WARN("trajectory callback in pthead");

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
bool queryTrajectoryStateService(
	pr2_controllers_msgs::QueryTrajectoryState::Request &req,
	pr2_controllers_msgs::QueryTrajectoryState::Response &resp)
{
	ROS_WARN("query state call in pthead");
	// Determines which segment of the trajectory to use
//	int seg = -1;
//	while (seg + 1 < (int)traj.size() &&
//			traj[seg+1].start_time < req.time.toSec()){
//	++seg;
//	}
//	if (seg == -1)
//		return false;

	resp.name.resize(number_of_joints_);
	resp.position.resize(number_of_joints_);
	resp.velocity.resize(number_of_joints_);
	resp.acceleration.resize(number_of_joints_);

	double timeval = time_ * dt_ + req.time.toSec() - ros::Time::now().toSec();

	for (unsigned int j = 0; j < number_of_joints_; ++j){
		resp.name[j] = joint_names_[j];
		resp.position[j] = vel_profile_[j].Pos(timeval);
		resp.velocity[j] = vel_profile_[j].Vel(timeval);
		resp.acceleration[j] = vel_profile_[j].Acc(timeval);
	}
	return true;
}

/**
* 
*/
int main(int argc, char **argv)
{

	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	ros::Subscriber twist_sub = n.subscribe("head_vel", 1, &twistCallback);
	ros::Subscriber traj_sub = n.subscribe("command", 1, &trajectoryCallback);
	ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("joint_states", 10);
	ros::Subscriber joy_sub = n.subscribe("psmove_out", 1, &joyCallback);
	ros::ServiceServer query_state_srv = n.advertiseService("query_state", &queryTrajectoryStateService);
	ros::Publisher c_state_pub = n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("state", 10);
	ros::Rate loop_rate(SPIN_FREQ);

	joint_state_msg.name.resize(2);
	joint_state_msg.name[0] = "head_pan_joint";
	joint_state_msg.name[1] = "head_tilt_joint";
	
	joint_state_msg.position.resize(2);
	joint_state_msg.position[0] = 0.0;
	joint_state_msg.position[1] = 0.0;
  
	joint_state_msg.velocity.resize(2);
	joint_state_msg.effort.resize(2);

	ROS_INFO("Starting node %s", nodeName.c_str());
	
	ptp = new PTProxy("/dev/ttyACM0");
	
	// Trajectory generation
    des_jnt_pos_.resize(number_of_joints_);
    vel_profile_.resize(number_of_joints_);
    joint_names_.resize(number_of_joints_);
	joint_names_[0] = "head_pan_joint";
	joint_names_[1] = "head_tilt_joint";
	q.resize(number_of_joints_);
	qd.resize(number_of_joints_);
	qdd.resize(number_of_joints_);
    trajectory_current_.positions.reserve(number_of_joints_);
    trajectory_current_.velocities.reserve(number_of_joints_);
    trajectory_current_.accelerations.reserve(number_of_joints_);
    trajectory_next_.positions.reserve(number_of_joints_);
    trajectory_next_.velocities.reserve(number_of_joints_);
    trajectory_next_.accelerations.reserve(number_of_joints_);
	trajectory_state_msg.desired.positions.resize(number_of_joints_);
	trajectory_state_msg.desired.velocities.resize(number_of_joints_);
	trajectory_state_msg.desired.accelerations.resize(number_of_joints_);
	trajectory_state_msg.actual.positions.resize(number_of_joints_);
	trajectory_state_msg.actual.velocities.resize(number_of_joints_);
	trajectory_state_msg.error.positions.resize(number_of_joints_);
	trajectory_state_msg.error.velocities.resize(number_of_joints_);
	q.resize(number_of_joints_);
	qd.resize(number_of_joints_);
	qdd.resize(number_of_joints_);
	meas_q.resize(number_of_joints_);
	meas_qd.resize(number_of_joints_);
	prev_meas_q.resize(number_of_joints_);
	dt_ = 1.0/SPIN_FREQ;

	int spinCnt = 1;
	while (ros::ok())
	{
		// Calculate desired trajectory point
		//q = ...

		// Get timestamp
		ros::Time now = ros::Time::now();
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
				float rx, ry;
				ptp->getJointPosition(rx, ry);

				for (size_t j = 0; j < number_of_joints_; ++j){
					prev_meas_q[j] = meas_q[j];
				}
				meas_q[0] = rx;
				meas_q[1] = ry;
				for (size_t j = 0; j < number_of_joints_; ++j){
					meas_qd[j] = (meas_q[j] - prev_meas_q[j]) * SPIN_FREQ;
				}

				// Building joint states message
				joint_state_msg.header.stamp = now;
				joint_state_msg.position[0] = rx;
				joint_state_msg.position[1] = ry;
				js_pub.publish(joint_state_msg);

				// Building controller state message
				trajectory_state_msg.header.stamp = now;
				for (size_t j = 0; j < number_of_joints_; ++j){
					trajectory_state_msg.desired.positions[j] = q[j];
					trajectory_state_msg.desired.velocities[j] = qd[j];
					trajectory_state_msg.desired.accelerations[j] = qdd[j];
					trajectory_state_msg.actual.positions[j] = meas_q[j];
					trajectory_state_msg.actual.velocities[j] = meas_qd[j];
					trajectory_state_msg.error.positions[j] = meas_q[j] - q[j];
					trajectory_state_msg.error.velocities[j] = meas_qd[j] - qd[j];
				}
				c_state_pub.publish(trajectory_state_msg);
			}
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
