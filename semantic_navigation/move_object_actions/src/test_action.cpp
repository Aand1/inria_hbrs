#include <move_object_actions/test_action.h>
#include <semantic_map/semantic_map.h>
#include <geometry_msgs/PoseStamped.h>  
#include <tf/transform_listener.h> 
#include <sem_nav_msgs/MoveObjectGoals.h>
#include <vector>

TestAction::TestAction(tf::TransformListener& tf, ros::NodeHandle &nh) :  tf_(tf), nh_(nh), planner_(NULL),runPlanner_(false), planner_costmap_ros_(NULL) 
{
	as_ = new PushActionServer(ros::NodeHandle(), "push_action", boost::bind(&TestAction::executeCb, this, _1), false);


    
	planner_ = new semantic_planner::SemanticPlannerGlobal( &tf_ );

	semantic_map_query_ = new semantic_map::SemanticMap(nh_);

	//makePlan();

	//set up the planner's thread
    //planner_thread_ = new boost::thread(boost::bind(&TestAction::planThread, this));


	
	as_->start();


    makePlan();
	
}

TestAction::~TestAction() 
{
	if(as_ != NULL)
      delete as_;

    planner_thread_->interrupt();
    planner_thread_->join();

    delete planner_thread_;

        
}

void TestAction::executeCb(const sem_nav_msgs::PushGoalConstPtr& push_goal)
{

    ROS_INFO_STREAM("Received request for pushing an object");

    //makePlan();

    sem_nav_msgs::PushGoal new_goal = *push_goal;
    sem_nav_msgs::MoveObjectGoals current_goals;
        
        //computeGoals(new_goal, current_goals);

        // initialize the states
        //push_action_states_ = APPROACHOBJECT;

    ros::NodeHandle n;
    //makePlan();
    while(nh_.ok())
    {
    	
        if(as_->isPreemptRequested())
        {
            if(as_->isNewGoalAvailable())
            {
              	ROS_INFO_STREAM("Received request for pushing an object");

              	sem_nav_msgs::PushGoal new_goal = *as_->acceptNewGoal();
                	//computeGoals(new_goal, current_goals);
                    // initialise states
                    //push_action_states_ = APPROACHOBJECT;
                                                                
            }
           
            else 
            {
                     //if we've been preempted explicitly we need to shut things down
                      //resetState();

                      //notify the ActionServer that we've successfully preempted
                ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
                as_->setPreempted();

                //we'll actually return from execute after preempting
                return;
            }
        }
      
        bool done = executeCycle(current_goals);   

        if (done)
        {
          	as_->setSucceeded(sem_nav_msgs::PushResult(), "Goal reached.");
            return;
        }

    }

    //if the node is killed then we'll abort and return
    as_->setAborted(sem_nav_msgs::PushResult(), "Aborting the action because the node has been killed");
    return;
}

bool TestAction::executeCycle(const sem_nav_msgs::MoveObjectGoals& goals)
{
	
    makePlan();
	return true;

}

void TestAction::makePlan()
{
	semantic_map::Object object;
	semantic_map_query_->getObject(object);
	ROS_INFO_STREAM(object);

	sem_nav_msgs::MoveObjectGoals object_goals;
	object_goals.approach_object.header.frame_id = "/map";
    object_goals.approach_object.pose.position.x = 2.5;
    object_goals.approach_object.pose.position.y = -1.0;

    geometry_msgs::PoseStamped robot_pose;
    robot_pose.header.stamp = ros::Time::now();
    robot_pose.header.frame_id = "map";
    robot_pose.pose.position.x = 3.5;
    robot_pose.pose.position.y = -2.0;
    robot_pose.pose.position.z = 0.0;
    robot_pose.pose.orientation.x = 0.0;
    robot_pose.pose.orientation.y = 0.0;
    robot_pose.pose.orientation.z = 0.0;
    robot_pose.pose.orientation.w = 0.0;

    std::vector<geometry_msgs::PoseStamped> plan;

	bool got_plan = planner_->makePlanObjectApproach(robot_pose, object_goals.approach_object, plan, object);

	ROS_INFO_STREAM(got_plan);
}

void TestAction::planThread()
{
	ROS_INFO_STREAM("running plan thread");
	ros::NodeHandle n;
	bool wait_for_wake = false;

	
		

    makePlan();



	

}
