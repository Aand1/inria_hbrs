
#include <move_robot_action/move_robot_action.h>


using namespace move_robot_action;

MoveRobotAction::MoveRobotAction(tf::TransformListener& tf) : 
    tf_(tf),
    as_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    planner_costmap_(NULL),
    controller_costmap_(NULL),
    runPlanner_(false), runController_(false),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL),
    initialized_(false)
{
	initialize();
}

MoveRobotAction::~MoveRobotAction() 
{

}

void MoveRobotAction::executeCb(const sem_nav_msgs::MoveRobotGoalConstPtr& move_robot_goal)
{
	//ROS_INFO_STREAM("In executecb");    
    if(!isQuaternionValid(move_robot_goal->target_pose.pose.orientation)){
      as_->setAborted(sem_nav_msgs::MoveRobotResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    ROS_INFO("move_robot action has received a new goal");
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_robot_goal->target_pose);
    
    //we have a goal so start the planner
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    planner_goal_ = goal;
    runPlanner_ = true;
    planner_cond_.notify_one();
    lock.unlock();

    std::vector<geometry_msgs::PoseStamped> global_plan;

    //we want to make sure that we reset the last time we had a valid plan and control
    last_valid_control_ = ros::Time::now();
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();

    ros::NodeHandle n;
    while(n.ok())
    {

      if(as_->isPreemptRequested())
      {
          if(as_->isNewGoalAvailable())
          {
            //if we're active and a new goal is available, we'll accept it, but we won't shut anything down
            sem_nav_msgs::MoveRobotGoal new_goal = *as_->acceptNewGoal();

            if(!isQuaternionValid(new_goal.target_pose.pose.orientation))
            {
              as_->setAborted(sem_nav_msgs::MoveRobotResult(), "Aborting on goal because it was sent with an invalid quaternion");
              return;
            }

            ROS_INFO("move_robot action has received a new goal");
            goal = goalToGlobalFrame(new_goal.target_pose);
            
            //Reset state for the next execution cycle
            state_ = PLANNING;

            //we have a new goal so make sure the planner is awake
            lock.lock();
            planner_goal_ = goal;
            runPlanner_ = true;
            planner_cond_.notify_one();
            lock.unlock();

            //make sure to reset our timeouts
            last_valid_control_ = ros::Time::now();
            last_valid_plan_ = ros::Time::now();
            last_oscillation_reset_ = ros::Time::now();


          }

          else 
          {
            //if we've been preempted explicitly we need to shut things down
            resetState();

            //notify the ActionServer that we've successfully preempted
            ROS_DEBUG_NAMED("move_base","Move base preempting the current goal");
            as_->setPreempted();

            //we'll actually return from execute after preempting
            return;
          }

      }

      //we also want to check if we've changed global frames because we need to transform our goal pose
      if(goal.header.frame_id != planner_costmap_->getGlobalFrameID())
      {
          //ROS_INFO_STREAM("Got a goal");
          goal = goalToGlobalFrame(goal);

          state_ = PLANNING;

          //we have a new goal so make sure the planner is awake
          lock.lock();
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          lock.unlock();

          //make sure to reset our timeouts
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();

      }

      
      //for timing that gives real time even in simulation
      ros::WallTime start = ros::WallTime::now();

      //the real work on pursuing a goal is done here
      bool done = executeCycle(goal, global_plan);

      //if we're done, then we'll return from execute
      if(done)
        return;

      //check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

     
    }

    //if the node is killed then we'll abort and return
    as_->setAborted(sem_nav_msgs::MoveRobotResult(), "Aborting on the goal because the node has been killed");
    resetState();
    return;

}

bool MoveRobotAction::executeCycle(geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan) 
{
	//we need to be able to publish velocity commands
	geometry_msgs::Twist cmd_vel;

	    
	//check that the observation buffers for the costmap are current, we don't want to drive blind
	if(!controller_costmap_->isCurrent())
	{
	    ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
	    publishZeroVelocity();
	    return false;
	}
    
    //if we have a new plan then grab it and give it to the controller
    if(new_global_plan_)
    {
	    //make sure to set the new plan flag to false
	    new_global_plan_ = false;

	    ROS_DEBUG_NAMED("move_base","Got a new plan...swap pointers");

	    //do a pointer swap under mutex
	    std::vector<geometry_msgs::PoseStamped>* temp_plan = controller_plan_;

	    boost::unique_lock<boost::mutex> lock(planner_mutex_);
	    controller_plan_ = latest_plan_;
	    latest_plan_ = temp_plan;
	    lock.unlock();
	    ROS_DEBUG_NAMED("move_base","pointers swapped!");
	  
        
        if(!controller_->setPlan(*controller_plan_))
        {
            //ABORT and SHUTDOWN COSTMAPS
            ROS_ERROR("Failed to pass global plan to the controller, aborting.");
            resetState();

            as_->setAborted(sem_nav_msgs::MoveRobotResult(), "Failed to pass global plan to the controller.");
            return true;
        }

      
    }

    
    //check to see if we've reached our goal
    if(controller_->isGoalReached())
    {

        ROS_INFO_STREAM("move_base, Goal reached!");
        resetState();

          //disable the planner thread
          boost::unique_lock<boost::mutex> lock(planner_mutex_);
          runPlanner_ = false;
          lock.unlock();
          state_ = STOP;

          as_->setSucceeded(sem_nav_msgs::MoveRobotResult(), "Goal reached.");
          return true;
        }
        
        {
         boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(controller_costmap_->getCostmap()->getMutex()));
        
        if(controller_->computeVelocityCommands(cmd_vel)){
          ROS_DEBUG_NAMED( "move_base", "Got a valid command from the local planner: %.3lf, %.3lf, %.3lf",
                           cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z );
          last_valid_control_ = ros::Time::now();
          //make sure that we send the velocity command to the base
          vel_pub_.publish(cmd_vel);
          
        }

        else{
        	state_ = PLANNING;
        }

      
    }
    

    //we aren't done yet
    return false;

}

void MoveRobotAction::initialize() 
{
	if(!initialized_)
    {

    	ros::NodeHandle private_nh("~");
        ros::NodeHandle nh;

        //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    	planner_costmap_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    	planner_costmap_->pause();

    	//create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    	controller_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    	controller_costmap_->pause();

    	//planner_ = new semantic_planner::SemanticPlannerGlobal( "global_planner", planner_costmap_ );

    	as_ = new MoveRobotActionServer(ros::NodeHandle(), "move_robot_action", boost::bind(&MoveRobotAction::executeCb, this, _1), false);

        ros::NodeHandle action_nh("move_robot_action");

        ros::NodeHandle simple_nh("move_robot_action_simple");


        //initializing some parameters that will be global to the move robot 
        std::string global_planner, local_planner;
        private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
        private_nh.param("planner_frequency", planner_frequency_, 1.0);
        private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
        private_nh.param("controller_frequency", controller_frequency_, 20.0);

        private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
        private_nh.param("global_costmap/global_frame", global_frame_, std::string("/map"));

        //initialize the global planner
        try 
        {
            planner_ = bgp_loader_.createInstance(global_planner);
            planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_);
        } 
        catch (const pluginlib::PluginlibException& ex) 
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
            exit(1);
        }

        //create a local planner
        try 
        {
            controller_ = blp_loader_.createInstance(local_planner);
            //ROS_INFO("Created local_planner %s", local_planner.c_str());
            controller_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_);
        } 
        catch (const pluginlib::PluginlibException& ex) 
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
            exit(1);
        }

        //set up plan buffer
        planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();
            
        //set up the planner's thread
        planner_thread_ = new boost::thread(boost::bind(&MoveRobotAction::planThread, this));

        //for comanding the base
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        //we're all set up now so we can start the action server
        as_->start();

        // Start actively updating costmaps based on sensor data
    	planner_costmap_->start();
    	//controller_costmap_->start();

        //initially, we'll need to make a plan
        state_ = PLANNING;

        initialized_ = false;


    }

}

void MoveRobotAction::planThread() 
{
	ros::NodeHandle n;
	ros::Timer timer;
	bool wait_for_wake = false;

	boost::unique_lock<boost::mutex> lock(planner_mutex_);

	while(n.ok())
	{
	    //check if we should run the planner (the mutex is locked)
	    while(wait_for_wake || !runPlanner_)
	    {
	        //if we should not be running the planner then suspend this thread
	        planner_cond_.wait(lock);
	        wait_for_wake = false;
	    }

	    ros::Time start_time = ros::Time::now();

	    geometry_msgs::PoseStamped temp_goal = planner_goal_;
	    lock.unlock();

	    //run planner
	    planner_plan_->clear();

	    bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);
	        

	    if(gotPlan)
	    {
	        std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;
	        temp_plan = planner_plan_;
	        lock.lock();
	        planner_plan_ = latest_plan_;
        	latest_plan_ = temp_plan;
        	new_global_plan_ = true;
	        lock.unlock();
	        
	        //publish the plan for visualization purposes
	        //publishPlan(*latest_plan_);

	        if(runPlanner_)
          	    state_ = CONTROLLING;

	    }

	    //take the mutex for the next iteration
	    lock.lock();
	        	        
	    //setup sleep interface for the planner
	    if(planner_frequency_ > 0)
	    {
	        ros::Duration sleep_time = (start_time + ros::Duration(1.0/planner_frequency_)) - ros::Time::now();
	        
	        if (sleep_time > ros::Duration(0.0))
	        {
	            wait_for_wake = true;
	            timer = n.createTimer(sleep_time, &MoveRobotAction::wakePlanner, this);
	        }
	    }

	}

}

bool MoveRobotAction::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) 
{
	boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_->getCostmap()->getMutex()));

	//make sure to set the plan to be empty initially
	plan.clear();

	//since this gets called on handle activate
	if(planner_costmap_ == NULL) {
	    ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
	    return false;
	}

	//get the starting pose of the robot
	tf::Stamped<tf::Pose> global_pose;
	if(!planner_costmap_->getRobotPose(global_pose)) 
	{
	    ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
	    return false;
	}

	geometry_msgs::PoseStamped start;
	tf::poseStampedTFToMsg(global_pose, start);

	//if the planner fails or returns a zero length plan, planning failed
	if(!planner_->makePlan(start, goal, plan) || plan.empty())
	{
	    ROS_DEBUG_NAMED("move_robot","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
	    return false;
	}
    return true;

}

void MoveRobotAction::wakePlanner(const ros::TimerEvent& event)
{
	// we have slept long enough for rate
	planner_cond_.notify_one();

}

bool MoveRobotAction::isQuaternionValid(const geometry_msgs::Quaternion& q) 
{
	//first we need to check if the quaternion has nan's or infs
    if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w))
    {
        ROS_ERROR("Quaternion has nans or infs... discarding as a navigation goal");
        return false;
    }

    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);

    //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6)
    {
        ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
        return false;
    }

    //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
    tf_q.normalize();

    tf::Vector3 up(0, 0, 1);

    double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

    if(fabs(dot - 1) > 1e-3)
    {
        ROS_ERROR("Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
        return false;
    }

    return true;

}

geometry_msgs::PoseStamped MoveRobotAction::goalToGlobalFrame(const geometry_msgs::PoseStamped& goal_pose_msg) 
{
	std::string global_frame = planner_costmap_->getGlobalFrameID();
    tf::Stamped<tf::Pose> goal_pose, global_pose;
  	poseStampedMsgToTF(goal_pose_msg, goal_pose);

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.stamp_ = ros::Time();

    try
    {
        tf_.transformPose(global_frame, goal_pose, global_pose);
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s", goal_pose.frame_id_.c_str(), global_frame.c_str(), ex.what());
        return goal_pose_msg;
    }

    geometry_msgs::PoseStamped global_pose_msg;
    tf::poseStampedTFToMsg(global_pose, global_pose_msg);
    return global_pose_msg;

}

void MoveRobotAction::publishZeroVelocity()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
}

void MoveRobotAction::resetState()
{
    // Disable the planner thread
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    runPlanner_ = false;
    lock.unlock();

    // Reset statemachine
    state_ = PLANNING;
    
    publishZeroVelocity();

}