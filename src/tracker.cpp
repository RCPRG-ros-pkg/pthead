#include "ros/ros.h"
#include <stdio.h>
#include <cstring>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"

#include <time.h>
#include <unistd.h>
#include <assert.h>

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"

#include <psmove.h>
#include <psmove_tracker.h>

// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

std::string nodeName = "ptmovetracker";

bool trackerPaused	= false;
bool trackerLog	= false;
bool pthSynchronize	= false;

/**
* 
*/
float inrange(float v, float mn, float mx) {
	return (v < mn ? mn : (v > mx ? mx : v) );
}

/**
* 
*/
int main(int argc, char **argv)
{
	geometry_msgs::PointStamped msgPoint;
	sensor_msgs::Joy		msgJoy;
	msgJoy.axes.resize(20);
	msgJoy.buttons.resize(17);

	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("head_vel", 10);
	ros::Publisher psmove_pub = n.advertise<sensor_msgs::Joy>("psmove_out", 10);
	ros::Publisher ballpos_pub = n.advertise<geometry_msgs::PointStamped>("ball_position", 10);
	ros::Rate loop_rate(50);

	ROS_INFO("Starting node %s", nodeName.c_str());
	
	// Opening Move controllers
    int i;
    int count = psmove_count_connected();
    PSMove* controllers[count];

    printf("### Found %d controllers.\n", count);
    if (count == 0) {
        return 1;
    }

    void *frame;
    int result;

    for (i=0; i<count; i++) {
        printf("Opening controller %d\n", i);
//        controllers[i] = psmove_connect();
        controllers[i] = psmove_connect_by_id(i);
        assert(controllers[i] != NULL);
    }
    
    fprintf(stderr, "Trying to init PSMoveTracker...");
    PSMoveTracker* tracker = psmove_tracker_new();
    psmove_tracker_set_mirror(tracker, PSMove_True);
    fprintf(stderr, "OK\n");

    for (i=0; i<count; i++) {
        while (1) {
            printf("Calibrating controller %d...", i);
            fflush(stdout);
            result = psmove_tracker_enable(tracker, controllers[i]);

            enum PSMove_Bool auto_update_leds =
                psmove_tracker_get_auto_update_leds(tracker, controllers[i]);
            if (result == Tracker_CALIBRATED) {
                printf("OK, auto_update_leds is %s\n",
                        (auto_update_leds == PSMove_True)?"enabled":"disabled");
                break;
            } else {
                printf("ERROR - retrying\n");
            }
        }
    }
	
	float dx = 0, dy = 0;
	
	bool quit = false;
	while (ros::ok() && (!quit)){
		// Update PSMove data
		int dupa = -10;
		while (psmove_poll(controllers[0])) {
			unsigned int pressed, released, buttons;
			float trigger, ax, ay, az, gx, gy, gz, mx, my, mz;
			psmove_get_button_events(controllers[0], &pressed, &released);
			if (pressed & Btn_MOVE) {
				trackerPaused = trackerPaused?false:true;
			} 
			if (pressed & Btn_CROSS) {
				pthSynchronize = true;
			} 
			else if (released & Btn_CROSS) {
				pthSynchronize = false;
			}
			
            // PSMove topic publish
			buttons = psmove_get_buttons(controllers[0]);
			trigger = psmove_get_trigger(controllers[0]);
			//psmove_get_accelerometer_frame(controllers[0], Frame_SecondHalf, &ax, &ay, &az);
			//psmove_get_gyroscope_frame(controllers[0], Frame_SecondHalf, &gx, &gy, &gz);
			
			msgJoy.header.stamp = ros::Time::now();
			
			msgJoy.buttons[PS3_BUTTON_SELECT] = (buttons & Btn_SELECT)?1:0;
			//msgJoy.buttons[PS3_BUTTON_STICK_LEFT] = (buttons & Btn_)?1:0;
			//msgJoy.buttons[PS3_BUTTON_STICK_RIGHT] = (buttons & Btn_)?1:0;
			msgJoy.buttons[PS3_BUTTON_START] = (buttons & Btn_START)?1:0;
			//msgJoy.buttons[PS3_BUTTON_CROSS_UP] = (buttons & Btn_)?1:0;
			//msgJoy.buttons[PS3_BUTTON_CROSS_RIGHT] = (buttons & Btn_)?1:0;
			//msgJoy.buttons[PS3_BUTTON_CROSS_DOWN] = (buttons & Btn_)?1:0;
			//msgJoy.buttons[PS3_BUTTON_CROSS_LEFT] = (buttons & Btn_)?1:0;
			//msgJoy.buttons[PS3_BUTTON_REAR_LEFT_2] = (buttons & Btn_)?1:0;
			msgJoy.buttons[PS3_BUTTON_REAR_RIGHT_2] = (buttons & Btn_T)?1:0;
			//msgJoy.buttons[PS3_BUTTON_REAR_LEFT_1] = (buttons & Btn_)?1:0;
			msgJoy.buttons[PS3_BUTTON_REAR_RIGHT_1] = (buttons & Btn_MOVE)?1:0;
			msgJoy.buttons[PS3_BUTTON_ACTION_TRIANGLE] = (buttons & Btn_TRIANGLE)?1:0;
			msgJoy.buttons[PS3_BUTTON_ACTION_CIRCLE] = (buttons & Btn_CIRCLE)?1:0;
			msgJoy.buttons[PS3_BUTTON_ACTION_CROSS] = (buttons & Btn_CROSS)?1:0;
			msgJoy.buttons[PS3_BUTTON_ACTION_SQUARE] = (buttons & Btn_SQUARE)?1:0;
			msgJoy.buttons[PS3_BUTTON_PAIRING] = (buttons & Btn_PS)?1:0;
			
			msgJoy.axes[PS3_AXIS_BUTTON_REAR_RIGHT_2] = trigger;
			
			psmove_pub.publish(msgJoy);
		}
		
		// Update tracker image
        psmove_tracker_update_image(tracker);
        psmove_tracker_update(tracker, NULL);
        frame = psmove_tracker_get_frame(tracker);

		msgPoint.header.stamp = ros::Time::now();

		//float limit = 70;
        float x, y, r;
        psmove_tracker_get_position(tracker, controllers[0], &x, &y, &r);
        //float xx = 0.05*(x-320);
        //float yy = 0.05*(y-240);
        //xx *= fabs(xx);
        //yy *= fabs(yy);

        //xx = inrange(xx, -limit, limit);
        //yy = inrange(yy, -limit, limit);
        
        //printf("x %5f y %5f\n", xx, yy);
		
		// Building message
		//msgTwist.angular.z = (trackerPaused?0.0:xx);
		//msgTwist.angular.y = (trackerPaused?0.0:yy);
        msgPoint.point.x = x;
        msgPoint.point.y = y;
		
		if(pthSynchronize) {
			pthSynchronize = false;
		}
		else {
			ballpos_pub.publish(msgPoint);
		}

		ros::spinOnce();

	//	loop_rate.sleep();
	}

	return 0;
}
