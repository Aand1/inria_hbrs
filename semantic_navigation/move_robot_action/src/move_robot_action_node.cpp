#include <move_robot_action/move_robot_action.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_robot_action_node");
	tf::TransformListener tf(ros::Duration(10));

	move_robot_action::MoveRobotAction move_robot_action( tf );
	
	//ros::MultiThreadedSpinner s;	
	ros::spin();
	
	return(0);
}
