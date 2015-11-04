#!/usr/bin/env python
# license removed for brevity

import rospy
import smach
import roslib; roslib.load_manifest('move_object_actions')
import smach_ros
from smach_ros import SimpleActionState, ServiceState
from smach import Sequence
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import String
from move_object_actions.msg import *
from std_msgs.msg import Bool
import tf



class ComputeGoals(smach.State) :
    def __init__(self) :
        smach.State.__init__(self, outcomes=['succeeded','failed'], input_keys=['compute_goals'], output_keys=['computed_goals'])
        self.counter = 0

    def execute(self, userdata) :
        #userdata.computed_goals
        push_object_goals = PushObjectGoals()
        push_object_goals.approach_object_goal.pose.position.x = 0.0
        push_object_goals.approach_object_goal.pose.position.y = -3.0
        push_object_goals.approach_object_goal.pose.orientation.w = -1.5

        push_object_goals.push_object_goal.header.frame_id = "/map";
        push_object_goals.push_object_goal.header.stamp = rospy.Time.now()
        push_object_goals.push_object_goal.pose.position.x = 0.17
        push_object_goals.push_object_goal.pose.position.y = -4.2
        push_object_goals.push_object_goal.pose.orientation.w = -0.3

        userdata.computed_goals = push_object_goals

        rate = rospy.Rate(10) # 10hz
        approach_object_goal_pub = rospy.Publisher('semantic_planner_global/goal', PushObjectGoals, queue_size=10, latch=True)

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

        #rospy.loginfo(userdata.goal.approach_object_goal.pose.orientation.w)
        quat = tf.transformations.quaternion_from_euler(0, userdata.goal.approach_object_goal.pose.orientation.z, userdata.goal.approach_object_goal.pose.orientation.w)
        goal.target_pose.pose.orientation = Quaternion(*quat)
        goal.target_pose.pose.position = Point(userdata.goal.approach_object_goal.pose.position.x, userdata.goal.approach_object_goal.pose.position.y, 0)
    
        return goal     


#define state GlobalPlannerTrigger
class GlobalPlannerTrigger(smach.State):
       def __init__(self):
           smach.State.__init__(self, outcomes=['succeeded'])
   
       def execute(self, userdata):
           trigger_global_planner_pub = rospy.Publisher('semantic_planner_global/trigger', Bool, queue_size=10, latch=True)
           rate = rospy.Rate(10)
           trigger = 1
           trigger_global_planner_pub.publish(trigger)

           return 'succeeded' 


#define state LocalPlannerTrigger
class LocalPlannerTrigger(smach.State):
       def __init__(self):
           smach.State.__init__(self, outcomes=['succeeded'])
   
       def execute(self, userdata):
           trigger_local_planner_pub = rospy.Publisher('semantic_planner_local/trigger', Bool, queue_size=10, latch=True)
           rate = rospy.Rate(10)
           trigger = 1
           trigger_local_planner_pub.publish(trigger)

           return 'succeeded'  

#define state StopRobot
class StopAction(smach.State):
       def __init__(self):
           smach.State.__init__(self, outcomes=['succeeded'])
   
       def execute(self, userdata):

           # Stop the Global planner
           stop_global_planner = rospy.Publisher('semantic_planner_local/trigger', Bool, queue_size=10, latch=True)
           rate = rospy.Rate(10)
           trigger = 0
           stop_global_planner.publish(trigger)

           # Stop the Local planner
           stop_local_planner = rospy.Publisher('semantic_planner_global/trigger', Bool, queue_size=10, latch=True)
           rate = rospy.Rate(10)
           trigger = 0
           stop_local_planner.publish(trigger)

           return 'succeeded'  
   

#define states for goal monitor
class goal_not_reached(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['goal_not_reached', 'preempted'])
    def execute(self, userdata):
        rospy.sleep(1.0)
        return 'goal_not_reached'
 
def monitor_cb(userdata, msg):

    return False

def child_term_cb(outcome_map):
    if outcome_map['Goal_Not_Reached'] == 'goal_not_reached':
        return True
    elif outcome_map['Goal_Reached'] == 'invalid':
        return True
    else:
        return False
 
def out_cb(outcome_map):
    if outcome_map['Goal_Reached'] == 'invalid':
         return 'goal_reached'
    elif outcome_map['Goal_Not_Reached'] == 'goal_not_reached':
         return 'goal_not_reached_yet'
    else:
         return 'goal_not_reached_yet'


def main() :
    rospy.init_node('push_action_cordinator')

    #Create SMACH state sequence
    push_action_sequence = Sequence(outcomes = ['succeeded', 'failed', 'aborted','preempted'], connector_outcome = 'succeeded')

    push_action_sequence.userdata.push_action_goals = PushObjectGoals()

    #Open the container
    with push_action_sequence :

        # compute goals state
        Sequence.add('COMPUTE_GOALS', ComputeGoals(), transitions={'succeeded':'MOVE_BASE_GO'}, remapping={'compute_goals':'push_action_goals', 'computed_goals':'push_action_goals'})

        # navigate to object 
        Sequence.add('MOVE_BASE_GO', MoveBaseState('/map'), transitions={'succeeded':'Semantic_Planner'}, remapping={'goal':'push_action_goals'})


        # Create the sub SMACH state machine 
        semantic_planner = Sequence(outcomes=['succeeded'], connector_outcome = 'succeeded')

        #Open the container
        with semantic_planner :

            # Generate global plan to push object
            Sequence.add('Global_Planner_Trigger', GlobalPlannerTrigger(), transitions={'succeeded':'Local_Planner_Trigger'})

            # Generate local plan to push object
            Sequence.add('Local_Planner_Trigger', LocalPlannerTrigger(), transitions={'succeeded':'succeeded'})

        Sequence.add('Semantic_Planner', semantic_planner, transitions={'succeeded':'Goal_Monitor'})  


        # Create the sub SMACH concurrence state machine for goal monitor
        goal_monitor = smach.Concurrence(outcomes=['goal_not_reached_yet', 'goal_reached'], default_outcome='goal_not_reached_yet', child_termination_cb=child_term_cb, outcome_cb=out_cb)   
        
        with goal_monitor:
            smach.Concurrence.add('Goal_Not_Reached', goal_not_reached())
            smach.Concurrence.add('Goal_Reached', smach_ros.MonitorState("/goal_monitor/goal_reached", Bool, monitor_cb))
 

        Sequence.add('Goal_Monitor', goal_monitor, transitions={'goal_not_reached_yet':'Semantic_Planner', 'goal_reached':'Stop_Action'})  

        Sequence.add('Stop_Action', StopAction(), transitions={'succeeded':'succeeded'})


    #Execute SMACH plan
    outcome = push_action_sequence.execute()


if __name__ == '__main__':
    main()
    rospy.spin()