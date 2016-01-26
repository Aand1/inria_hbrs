#include <ros/ros.h>
#include <vector>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "gazebo/physics/State.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/WorldState.hh"
#include "gazebo/physics/ModelState.hh"

#include "gazebo/common/Exception.hh"
#include <boost/bind.hpp>
#include <geometry_msgs/Vector3.h> 
#include <perception/ObjectStates.h>
#include <perception/ObjectState.h>





using namespace gazebo;
using namespace physics;

namespace gazebo
{
    class PerceptionSimulation : public WorldPlugin, public State
    {
        public: PerceptionSimulation(){}
	public: ~PerceptionSimulation(){}

        public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
// Store the pointer to the model
             this->model = _world;

             this->namespace_ =_world->GetName ();
             if ( !namespace_.empty() ) this->namespace_ += "/";
             rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle ( this->namespace_ ) );

             object_size_publisher_ = rosnode_->advertise<perception::ObjectStates> ( "object_size",1 ,true);
	     
            // Listen to the update event. This event is broadcast every
            // simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&PerceptionSimulation::OnUpdate, this, _1));
        }

	public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    	{
	    publishObjectSize();
            
	}
   
        public: void publishObjectSize()
	{
             geometry_msgs::Vector3 size;
             perception::ObjectStates object_states;
             
	     // Add a state for all the models
             Model_V models = model->GetModels();
             
             //ROS_INFO_STREAM(models[0]->GetBoundingBox());
             //this->modelStates.clear();
             for (Model_V::const_iterator iter = models.begin();iter != models.end(); ++iter)
             {
                 perception::ObjectState object_state;
                 gazebo::physics::ModelPtr model = *iter;

                 model->GetName();  

                 ////////////////////////////////////////////

	/*	 object_state.pose.orientation.x = pose.rot.x;
		 object_state.pose.orientation.y = pose.rot.y;
		 object_state.pose.orientation.z = pose.rot.z;
                 object_state.pose.orientation.w = pose.rot.w;			
		 	
                 math::Box box = model->GetBoundingBox();

                 math::Vector3 bb_size = box.GetSize();
                 object_state.size.x = bb_size.x;
                 object_state.size.y = bb_size.y;
                 object_state.size.z = bb_size.z;

                 //object_states.Object_states.push_back(object_state);*/

		
            	// size.x = bb_size.x;
            	 //size.y = bb_size.y;rintf("%f \n", bb.GetSize());date_rate_;
	     }
        }
        private: double update_period_;

        private: std::string namespace_;
        private: boost::shared_ptr<ros::NodeHandle> rosnode_;
	private: ros::Publisher object_size_publisher_; 
        
    };


GZ_REGISTER_WORLD_PLUGIN(PerceptionSimulation)
}
