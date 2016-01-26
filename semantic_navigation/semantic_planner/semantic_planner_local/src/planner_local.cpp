#include "semantic_planner_local/planner_local.h"

using namespace semantic_planner;

PlannerLocal::PlannerLocal() 
{
   	
}

PlannerLocal::PlannerLocal(std::string name,
                           tf::TransformListener* tf, ros::NodeHandle &nh,
                           costmap_2d::Costmap2DROS* costmap_ros) :
    world_model_(NULL), tc_(NULL), costmap_ros_(NULL), tf_(NULL), setup_(false), initialized_(false), odom_helper_("odom")
{
	//initialize the planner
    initialize(name, tf, nh, costmap_ros);
}

void PlannerLocal::initialize(std::string name, tf::TransformListener* tf, ros::NodeHandle &nh, costmap_2d::Costmap2DROS* costmap_ros) 
{

	if (! isInitialized()) 
	{
		tf_ = tf;
		nh_ = nh;
		costmap_ros_ = costmap_ros;

		//initialize the copy of the costmap the controller will use
        costmap_ = costmap_ros_->getCostmap();

        global_frame_ = costmap_ros_->getGlobalFrameID();
        robot_base_frame_ = costmap_ros_->getBaseFrameID();


        ros::NodeHandle private_nh("~/" + name);

        local_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

        //for comanding the base
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        double sim_time, sim_granularity, angular_sim_granularity;
	    int vx_samples, vtheta_samples;
	    double pdist_scale, gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta;
	    bool holonomic_robot, dwa, simple_attractor, heading_scoring;
	    double heading_scoring_timestep;
	    double max_vel_x, min_vel_x;
	    double backup_vel;
	    double stop_time_buffer;
	    std::string world_model_type;
	    std::string controller_frequency_param_name;
	    bool meter_scoring;
	    double max_rotational_vel;
	    double min_pt_separation, max_obstacle_height, grid_resolution;
	    rotating_to_goal_ = false;
	    std::vector<double> y_vels = loadYVels(private_nh);

        // Initializing parameters
        private_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);
	    private_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
	    private_nh.param("acc_lim_x", acc_lim_x_, 2.5);
	    private_nh.param("acc_lim_y", acc_lim_y_, 2.5);
	    private_nh.param("acc_lim_theta", acc_lim_theta_, 3.2);

        private_nh.param("stop_time_buffer", stop_time_buffer, 0.2);

        private_nh.param("latch_xy_goal_tolerance", latch_xy_goal_tolerance_, false);

        //Since I screwed up nicely in my documentation, I'm going to add errors
        //informing the user if they've set one of the wrong parameters
        if(private_nh.hasParam("acc_limit_x"))
            ROS_ERROR("You are using acc_limit_x where you should be using acc_lim_x. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

        if(private_nh.hasParam("acc_limit_y"))
            ROS_ERROR("You are using acc_limit_y where you should be using acc_lim_y. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

        if(private_nh.hasParam("acc_limit_th"))
            ROS_ERROR("You are using acc_limit_th where you should be using acc_lim_th. Please change your configuration files appropriately. The documentation used to be wrong on this, sorry for any confusion.");

        if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
            sim_period_ = 0.05;
        else
        {
        	double controller_frequency = 0;
        	private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
        	if(controller_frequency > 0)
            	sim_period_ = 1.0 / controller_frequency;
        	else
        	{
          		ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
          		sim_period_ = 0.05;
        	}
        }
      
        ROS_INFO("Sim period is set to %.2f", sim_period_);

      	private_nh.param("sim_time", sim_time, 1.0);
      	private_nh.param("sim_granularity", sim_granularity, 0.025);
      	private_nh.param("angular_sim_granularity", angular_sim_granularity, sim_granularity);
      	private_nh.param("vx_samples", vx_samples, 3);
      	private_nh.param("vtheta_samples", vtheta_samples, 20);

      	private_nh.param("path_distance_bias", pdist_scale, 0.6);
      	private_nh.param("goal_distance_bias", gdist_scale, 0.8);
      	private_nh.param("occdist_scale", occdist_scale, 0.01);

      	if ( ! private_nh.hasParam("meter_scoring")) 
      	{
        	ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring not set. Set it to true to make your settins robust against changes of costmap resolution.");
        } 
        else 
        {
        	private_nh.param("meter_scoring", meter_scoring, false);

        	if(meter_scoring) 
        	{
            	//if we use meter scoring, then we want to multiply the biases by the resolution of the costmap
          		double resolution = costmap_->getResolution();
          		gdist_scale *= resolution;
          		pdist_scale *= resolution;
          		occdist_scale *= resolution;
            } 
            else 
            {
            	ROS_WARN("Trajectory Rollout planner initialized with param meter_scoring set to false. Set it to true to make your settins robust against changes of costmap resolution.");
        	}
      	}

      	private_nh.param("heading_lookahead", heading_lookahead, 0.325);
      	private_nh.param("oscillation_reset_dist", oscillation_reset_dist, 0.05);
	    private_nh.param("escape_reset_theta", escape_reset_theta, M_PI_4);
     	private_nh.param("holonomic_robot", holonomic_robot, true);
      	private_nh.param("max_vel_x", max_vel_x, 0.5);
      	private_nh.param("min_vel_x", min_vel_x, 0.1);

      	private_nh.param("max_rotational_vel", max_rotational_vel, 1.0);
      	private_nh.param("min_in_place_rotational_vel", min_in_place_vel_th_, 0.4);

      	if(private_nh.getParam("backup_vel", backup_vel))
        	ROS_WARN("The backup_vel parameter has been deprecated in favor of the escape_vel parameter. To switch, just change the parameter name in your configuration files.");

      	//if both backup_vel and escape_vel are set... we'll use escape_vel
      	private_nh.getParam("escape_vel", backup_vel);

      	if(backup_vel >= 0.0)
        	ROS_WARN("You've specified a positive escape velocity. This is probably not what you want and will cause the robot to move forward instead of backward. You should probably change your escape_vel parameter to be negative");

      	private_nh.param("world_model", world_model_type, std::string("costmap"));
      	private_nh.param("dwa", dwa, true);
      	private_nh.param("heading_scoring", heading_scoring, false);
      	private_nh.param("heading_scoring_timestep", heading_scoring_timestep, 0.8);

      	//parameters for using the freespace controller
     	private_nh.param("point_grid/max_sensor_range", max_sensor_range_, 2.0);
      	private_nh.param("point_grid/min_pt_separation", min_pt_separation, 0.01);
      	private_nh.param("point_grid/max_obstacle_height", max_obstacle_height, 2.0);
      	private_nh.param("point_grid/grid_resolution", grid_resolution, 0.2);

      	ROS_ASSERT_MSG(world_model_type == "costmap", "At this time, only costmap world models are supported by this controller");
      	world_model_ = new base_local_planner::CostmapModel(*costmap_);
      	
      	footprint_spec_ = costmap_ros_->getRobotFootprint();

      	tc_ = new base_local_planner::TrajectoryPlanner(*world_model_, *costmap_, footprint_spec_,
          acc_lim_x_, acc_lim_y_, acc_lim_theta_, sim_time, sim_granularity, vx_samples, vtheta_samples, pdist_scale,
          gdist_scale, occdist_scale, heading_lookahead, oscillation_reset_dist, escape_reset_dist, escape_reset_theta, holonomic_robot,
          max_vel_x, min_vel_x, max_vel_th_, min_vel_th_, min_in_place_vel_th_, backup_vel,
          dwa, heading_scoring, heading_scoring_timestep, meter_scoring, simple_attractor, y_vels, stop_time_buffer, sim_period_, angular_sim_granularity);

      	map_viz_.initialize(name, global_frame_, boost::bind(&base_local_planner::TrajectoryPlanner::getCellCosts, tc_, _1, _2, _3, _4, _5, _6));

      	ROS_INFO_STREAM("Local Planner initialised");

      	initialized_ = true;

	}

}

PlannerLocal::~PlannerLocal()
{

}

bool PlannerLocal::sendVelocityCommands()
{ 
    geometry_msgs::Twist cmd_vel;

    if( computeVelocityCommands(cmd_vel) )
    {
        vel_pub_.publish(cmd_vel);

        return true;
    }

    else
        return false;

}

void PlannerLocal::stop()
{ 
    geometry_msgs::Twist cmd_vel;

    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;

    
    vel_pub_.publish(cmd_vel);
}

std::vector<double> PlannerLocal::loadYVels(ros::NodeHandle node)
{
    std::vector<double> y_vels;

    std::string y_vel_list;
    if(node.getParam("y_vels", y_vel_list))
    {
      	typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
      	boost::char_separator<char> sep("[], ");
      	tokenizer tokens(y_vel_list, sep);

	      for(tokenizer::iterator i = tokens.begin(); i != tokens.end(); i++)
	      {
	        	y_vels.push_back(atof((*i).c_str()));
	      }
    }
    else
    {
	      //if no values are passed in, we'll provide defaults
	      y_vels.push_back(-0.3);
	      y_vels.push_back(-0.1);
	      y_vels.push_back(0.1);
	      y_vels.push_back(0.3);
    }

    return y_vels;
}
