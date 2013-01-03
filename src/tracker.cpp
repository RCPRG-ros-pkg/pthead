#include "ros/ros.h"
#include <stdio.h>
#include <cstring>
#include "geometry_msgs/Twist.h"

#include <time.h>
#include <unistd.h>
#include <assert.h>

#include "opencv2/core/core_c.h"
#include "opencv2/highgui/highgui_c.h"

#include <psmove.h>
#include <psmove_tracker.h>

std::string nodeName = "ptmovetracker";

float inrange(float v, float mn, float mx) {
	return (v < mn ? mn : (v > mx ? mx : v) );
}

void
wait_for_button(PSMove *move, int button)
{
    /* Wait for press */
    while ((psmove_get_buttons(move) & button) == 0) {
        psmove_poll(move);
        psmove_update_leds(move);
    }

    /* Wait for release */
    while ((psmove_get_buttons(move) & button) != 0) {
        psmove_poll(move);
        psmove_update_leds(move);
    }
}

/**
* 
*/
int main(int argc, char **argv)
{
	geometry_msgs::Twist	msgTwist;

	ros::init(argc, argv, nodeName);
	ros::NodeHandle n;

	ros::Publisher js_pub = n.advertise<geometry_msgs::Twist>("/head_vel", 10);
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
		// Prepare data to send
        psmove_tracker_update_image(tracker);
        psmove_tracker_update(tracker, NULL);
        frame = psmove_tracker_get_frame(tracker);

		float limit = 70;
        float x, y, r;
        psmove_tracker_get_position(tracker, controllers[0], &x, &y, &r);
        float xx = 0.05*(x-320);
        float yy = 0.05*(y-240);
        xx *= fabs(xx);
        yy *= fabs(yy);

        xx = inrange(xx, -limit, limit);
        yy = inrange(yy, -limit, limit);

        printf("x: %10.2f, y: %10.2f, r: %10.2f\n", xx, yy, r);
		
		// Building message
		msgTwist.angular.z = xx;
		msgTwist.angular.y = yy;
		js_pub.publish(msgTwist);

		ros::spinOnce();

	//	loop_rate.sleep();
	}

	return 0;
}
