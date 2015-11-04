#!/usr/bin/env python
# license removed for brevity

import roslib; roslib.load_manifest('move_object_actions')
import rospy
import smach
from smach_ros import SimpleActionState, ServiceState
from smach import Sequence
import tf

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from move_object_actions.msg import *
from std_msgs.msg import Bool




#define state Sleep
class Sleep(smach.State):
       def __init__(self):
           smach.State.__init__(self, outcomes=['sleep_succeded'])
   
       def execute(self, userdata):
           rospy.loginfo('Sleeping for a while')
           rospy.sleep(10)
           return 'sleep_succeded'


class ComputeGoals(smach.State) :
	def __init__(self) :
		smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['compute_goals'], output_keys=['computed_goals'])
		self.counter = 0

	def execute(self, userdata) :
		#userdata.computed_goals
		push_object_goals = PushObjectGoals()
		push_object_goals.approach_object_goal.pose.position.x = 0.0
		push_object_goals.approach_object_goal.pose.position.y = -3.4
		push_object_goals.approach_object_goal.pose.orientation.w = -1.5

		push_object_goals.push_object_goal.header.frame_id = "/map";
    	push_object_goals.push_object_goal.header.stamp = rospy.Time.now()
    	push_object_goals.push_object_goal.pose.position.x = 2.0#0.0;
    	push_object_goals.push_object_goal.pose.position.y = 1.0#-6.0;



		#push_object_goals.approach_object_goal.pose.orientation.z = -0.7

		#userdata.computed_goals.approach_object_goal.header.frame_id = '/map'
		#userdata.computed_goals.approach_object_goal.header.stamp = rospy.Time.now()
		#computed_goals.approach_object_goal.pose.position.x = 2.0
		#computed_goals.approach_object_goal.pose.position.y = 2.0
		#computed_goals.approach_object_goal.pose.orientation.w = 1.0
		#userdata.compute_goals.approach_object_goal.header.frame_id = '/map'
		#userdata.computed_goals.approach_object_goal.header.frame_id = userdata.compute_goals.approach_object_goal.header.frame_id
		userdata.computed_goals = push_object_goals

		rate = rospy.Rate(10) # 10hz
		approach_object_goal_pub = rospy.Publisher('semantic_planner_global/goal', PushObjectGoals, queue_size=10, latch=True)
		#while approach_object_goal_pub.get_num_connections() == 0:
        #		rate.sleep()

		approach_object_goal_pub.publish(push_object_goals)

		return 'succeeded'


class MoveBaseState(SimpleActionState):
    """Calls a move_base action server with the goal (x, y, yaw) from userdata"""
    def __init__(self, frame='/map'):
        SimpleActionState.__init__(self, 'move_base', MoveBaseAction, input_keys=['goal'], goal_cb=self.__goal_cb)
        self.frame = frame

    def __goal_cb(self, userdata, old_goal):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame
        goal.target_pose.header.stamp = rospy.Time.now()

        rospy.loginfo(userdata.goal.approach_object_goal.pose.orientation.w)
        quat = tf.transformations.quaternion_from_euler(0, userdata.goal.approach_object_goal.pose.orientation.z, userdata.goal.approach_object_goal.pose.orientation.w)
        goal.target_pose.pose.orientation = Quaternion(*quat)
        goal.target_pose.pose.position = Point(userdata.goal.approach_object_goal.pose.position.x, userdata.goal.approach_object_goal.pose.position.y, 0)
	

        return goal

class GlobalPlannerTrigger(smach.State):
	def __init__(self) :
    		smach.State.__init__(self, outcomes=['global_planner_trigger_succeeded'])

	def execute(self, userdata):
        #trigger_global_planner_pub = rospy.Publisher('semantic_planner_global/trigger', Bool, queue_size=10)
    	#rate = rospy.Rate(10) # 10hz

    	#trigger = 1
 		#trigger_global_planner_pub.publish(trigger)
          
		return 'global_planner_trigger_succeeded'



def main() :
	rospy.init_node('smach_example_sm')

	#Create SMACH state machine
	sq = Sequence(outcomes = ['succeeded', 'failed', 'aborted','preempted'], connector_outcome = 'succeeded')

	sq.userdata.goal_position_x = 2
    	sq.userdata.goal_position_y = 0
    	sq.userdata.goal_position_yaw = 1
    	sq.userdata.saved_position_x = -2
    	sq.userdata.saved_position_y = -3
    	sq.userdata.saved_position_yaw = 1

	sq.userdata.sq_goals = PushObjectGoals()


	#Open the container
	with sq :
		# nav to goal
        	#Sequence.add('MOVE_BASE_GO', MoveBaseState('/map'), transitions={'succeeded':'FOO'}, remapping={'x':'goal_position_x', 'y':'goal_position_y', 'yaw':'goal_position_yaw'})

		# compute goals state
		Sequence.add('COMPUTE_GOALS', ComputeGoals(), transitions={'succeeded':'MOVE_BASE_GO'}, remapping={'compute_goals':'sq_goals', 'computed_goals':'sq_goals'})

		# navigate to object 
		Sequence.add('MOVE_BASE_GO', MoveBaseState('/map'), transitions={'succeeded':'Global_Planner_Trigger'}, remapping={'goal':'sq_goals'})

		# Generate global plan to push object
		Sequence.add('Global_Planner_Trigger', GlobalPlannerTrigger(), transitions={'global_planner_trigger_succeeded':'Sleep'})

		# Sleep for a while
		Sequence.add('Sleep', Sleep(), transitions={'sleep_succeded':'succeeded'})
		# nav back
        	#Sequence.add('MOVE_BASE_RETURN', MoveBaseState('/map'), transitions={'succeeded':'succeeded'}, remapping={'x':'saved_position_x', 'y':'saved_position_y', 'yaw':'saved_position_yaw'})


	#Execute SMACH plan
	outcome = sq.execute()

if __name__ == '__main__':
	main()
	rospy.spin()

