#include <move_object_actions/tap_action.h>
#include <sem_nav_msgs/MoveObjectGoals.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "tap_action");	
	//ros::NodeHandle nh;

	tf::TransformListener tf(ros::Duration(10));

	//TestAction* test_action = new TestAction( tf, nh );
	move_object::TapAction tap_action(tf);
        
	//ros::MultiThreadedSpinner s;
        ros::spin();

	return 0;

}

