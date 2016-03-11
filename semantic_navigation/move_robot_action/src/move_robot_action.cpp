
#include <move_robot_action/move_robot_action.h>


using namespace move_robot_action;

MoveRobotAction::MoveRobotAction(tf::TransformListener& tf) : 
    tf_(tf),
    as_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner"),
    global_costmap_(NULL),
    local_costmap_(NULL),
    runPlanner_(false), runController_(false),
    planner_plan_(NULL), latest_plan_(NULL), controller_plan_(NULL), goal_monitor_(NULL),
    initialized_(false)
{
	initialize();
}

MoveRobotAction::~MoveRobotAction() 
{

}

void MoveRobotAction::executeCb(const sem_nav_msgs::MoveRobotGoalConstPtr& move_robot_goal)
{
	  ROS_INFO_STREAM(ros::Time::now());    
    if(!isQuaternionValid(move_robot_goal->target_pose.pose.orientation)){
      as_->setAborted(sem_nav_msgs::MoveRobotResult(), "Aborting on goal because it was sent with an invalid quaternion");
      return;
    }

    ROS_INFO("move_robot action has received a new goal");
    geometry_msgs::PoseStamped goal = goalToGlobalFrame(move_robot_goal->target_pose);

    //updateSemanticCostmapConstraints(constraints_.semantic_costmap_constraints);
    //updateLocalPlannerConstraints(constraints_.local_planner_constraints);

    global_costmap_->start();
    local_costmap_->start();
    
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

            //updateSemanticCostmapConstraints(constraints_.semantic_costmap_constraints);
            //updateLocalPlannerConstraints(constraints_.local_planner_constraints);

            global_costmap_->start();
            local_costmap_->start();

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
      if(goal.header.frame_id != global_costmap_->getGlobalFrameID())
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
      { 
        ROS_INFO_STREAM(ros::Time::now());
        return;
      }

      //check if execution of the goal has completed in some way

      ros::WallDuration t_diff = ros::WallTime::now() - start;
      ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

     
    }

    //if the node is killed then we'll abort and return
    as_->setAborted(sem_nav_msgs::MoveRobotResult(), "Aborting on the goal because the node has been killed");
    resetState();
    return;

}

bool MoveRobotAction::executeCycle(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& global_plan) 
{
  	//we need to be able to publish velocity commands
  	geometry_msgs::Twist cmd_vel;
    const geometry_msgs::PoseStamped goal_;
    //ROS_INFO_STREAM(goal);

  	    
  	//check that the observation buffers for the costmap are current, we don't want to drive blind
  	if(!local_costmap_->isCurrent())
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


    switch(state_)
    {
        case PLANNING:
        {
            boost::mutex::scoped_lock lock(planner_mutex_);
            runPlanner_ = true;
            planner_cond_.notify_one();

        }
        break;

        case CONTROLLING:
        { 
            double tolerance = 0.15;
            if ( goal_monitor_->goalMonitor(goal, tolerance) )
            //if ( 1 )
            {

                resetState();

                //disable the planner thread
                boost::unique_lock<boost::mutex> lock(planner_mutex_);
                runPlanner_ = false;
                lock.unlock();

                as_->setSucceeded(sem_nav_msgs::MoveRobotResult(), "Goal reached.");
                return true;

            }

            else
            {
                controller_->sendVelocityCommands();
            }  

            //check for an oscillation condition
          /*  if(oscillation_timeout_ > 0.0 &&
                last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
            {
                publishZeroVelocity();
            }*/

            /*{
                boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(local_costmap_->getCostmap()->getMutex()));

                if (controller_->sendVelocityCommands() )
                {
                    last_valid_control_ = ros::Time::now();
                }

                else 
                {
                    ROS_DEBUG_NAMED("move_robot_action", "The local planner could not find a valid plan.");
                    ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

                    //check if we've tried to find a valid control for longer than our time limit
                    if(ros::Time::now() > attempt_end)
                    {
                        //we'll move into our obstacle clearing mode
                        //publishZeroVelocity();

                        as_->setAborted(sem_nav_msgs::MoveRobotResult(), "Failed to find a valid control.");
                        
                    }
                    else
                    {
                        //otherwise, if we can't find a valid control, we'll go back to planning
                        last_valid_plan_ = ros::Time::now();
                        //state_ = PLANNING;
                        //publishZeroVelocity();

                        //enable the planner thread in case it isn't running on a clock
                        boost::unique_lock<boost::mutex> lock(planner_mutex_);
                        runPlanner_ = true;
                        planner_cond_.notify_one();
                        lock.unlock();
                    }

                }       
        
            }*/
              

        }
        break;

        
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
    	  global_costmap_ = new semantic_costmap::SemanticCostmapROS("global_costmap", tf_);
    	  global_costmap_->pause();

    	  //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    	  local_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    	  local_costmap_->pause();

    	  //planner_ = new semantic_planner::SemanticPlannerGlobal( "global_planner", planner_costmap_ );

    	  as_ = new MoveRobotActionServer(ros::NodeHandle(), "move_robot_action", boost::bind(&MoveRobotAction::executeCb, this, _1), false);

        //initializing some parameters that will be global to the move robot 
        std::string global_planner, local_planner;
        private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
        private_nh.param("planner_frequency", planner_frequency_, 1.0);
        private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
        private_nh.param("controller_frequency", controller_frequency_, 20.0);

        private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
        private_nh.param("global_costmap/global_frame", global_frame_, std::string("/map"));

        //initialize the global planner
        planner_ = new semantic_planner::SemanticPlannerGlobal( "global_planner" , global_costmap_);
        controller_ = new semantic_planner::SemanticPlannerLocal( "local_planner", &tf_, local_costmap_ );
        goal_monitor_ = new semantic_planner::GoalMonitor();
        /*try 
        {
            planner_ = bgp_loader_.createInstance(global_planner);
            planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_);
        } 
        catch (const pluginlib::PluginlibException& ex) 
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
            exit(1);
        }*/

        //create a local planner
      /*  try 
        {
            controller_ = blp_loader_.createInstance(local_planner);
            //ROS_INFO("Created local_planner %s", local_planner.c_str());
            controller_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_);
        } 
        catch (const pluginlib::PluginlibException& ex) 
        {
            ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
            exit(1);
        }*/

        //set up plan buffer
        planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        latest_plan_ = new std::vector<geometry_msgs::PoseStamped>();
        controller_plan_ = new std::vector<geometry_msgs::PoseStamped>();
            
        //set up the planner's thread
        planner_thread_ = new boost::thread(boost::bind(&MoveRobotAction::planThread, this));

        //for comanding the base
        vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        ros::NodeHandle action_nh("move_robot_action");
        action_goal_pub_ = action_nh.advertise<sem_nav_msgs::MoveRobotActionGoal>("goal", 1);

        ros::NodeHandle simple_nh("move_base_simple");
        goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveRobotAction::goalCB, this, _1));

        //subscribe to update constraints
        constraints_sub_ = nh.subscribe<sem_nav_msgs::Constraints>("constraints", 1, boost::bind(&MoveRobotAction::constraintsCB, this, _1));

        //advertise a service for getting a plan
        make_plan_srv_ = private_nh.advertiseService("make_plan", &MoveRobotAction::planService, this);

        //we're all set up now so we can start the action server
        as_->start();

        // Start actively updating costmaps based on sensor data
    	  global_costmap_->startCostmap();
    	  local_costmap_->start();

        // Stop costmap
        //global_costmap_->stopCostmap();
        //local_costmap_->stop();

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
	boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(global_costmap_->getCostmap()->getMutex()));

	//make sure to set the plan to be empty initially
	plan.clear();

	//since this gets called on handle activate
	if(global_costmap_ == NULL) {
	    ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
	    return false;
	}

	//get the starting pose of the robot
	tf::Stamped<tf::Pose> global_pose;
	if(!global_costmap_->getRobotPose(global_pose)) 
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

  /*  //next, we need to check if the length of the quaternion is close to zero
    if(tf_q.length2() < 1e-6)
    {
        ROS_ERROR("Quaternion has length close to zero... discarding as navigation goal");
        return false;
    }*/

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
	  std::string global_frame = global_costmap_->getGlobalFrameID();
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

void MoveRobotAction::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
    ROS_DEBUG_NAMED("move_robot_action","In ROS goal callback, wrapping the PoseStamped in the action message and re-sending to the server.");
    ROS_INFO_STREAM("RECEIVED GOAL FROM RVIZ");
    sem_nav_msgs::MoveRobotActionGoal action_goal;
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal.target_pose = *goal;

    global_costmap_->startCostmap();
    action_goal_pub_.publish(action_goal);
}

bool MoveRobotAction::planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp)
{

    if(as_->isActive())
    {
      ROS_ERROR("move_robot must be in an inactive state to make a plan for an external user");
      return false;
    }

    global_costmap_->startCostmap();

    //make sure we have a costmap for our planner
    if(global_costmap_ == NULL)
    {
      ROS_ERROR("move_robot cannot make a plan for you because it doesn't have a costmap");
      return false;
    }
    tf::Stamped<tf::Pose> global_pose;
    if(!global_costmap_->getRobotPose(global_pose))
    {
      ROS_ERROR("move_base cannot make a plan for you because it could not get the start pose of the robot");
      return false;
    }
    
    std::vector<geometry_msgs::PoseStamped> plan;
    req.goal.header.frame_id = "/map";

    makePlan(req.goal, plan); 

    //copy all the plan into a message to send out
    resp.plan.poses.resize(plan.size());

    for(unsigned int i = 0; i < plan.size(); ++i)
    {
        resp.plan.poses.push_back(plan[i]); 
    }

    global_costmap_->stop();

    return true;

}

void MoveRobotAction::localizationCb(const sem_nav_msgs::SemanticPose::ConstPtr& pose)
{
    robot_pose = *pose;   
}

void MoveRobotAction::constraintsCB(const sem_nav_msgs::Constraints::ConstPtr& constraints)
{
    //updateLocalPlannerConstraints(constraints->local_planner_constraints);
    constraints_ = *constraints;
    //ROS_INFO_STREAM(constraints_.local_planner_constraints);
}

void MoveRobotAction::updateLocalPlannerConstraints(const sem_nav_msgs::LocalPlannerConstraints& lpc)
{

    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;



    for(int i = 0; i < lpc.doubles.size(); i++)
    {
        //ROS_INFO_STREAM("Updating constraints for local planner");

        conf.doubles.push_back(lpc.doubles[i]);
    }

  /*  double_param.name = "sim_time";
    double_param.value = 0.7;
    conf.doubles.push_back(double_param);

    double_param.name = "max_vel_x";
    double_param.value = 0.7;
    conf.doubles.push_back(double_param);*/

    srv_req.config = conf;

    if ( ros::service::call("/move_robot_action/TrajectoryPlannerROS/set_parameters", srv_req, srv_resp) )
    {
        //ROS_INFO_STREAM("Service calling successful");
    }

}

void MoveRobotAction::updateSemanticCostmapConstraints(const sem_nav_msgs::SemanticCostmapConstraints& semantic_costmap_constraints)
{
    ROS_INFO_STREAM("Updating semantic costmap constraints from move robot action");
    global_costmap_->setParameters(semantic_costmap_constraints);
}