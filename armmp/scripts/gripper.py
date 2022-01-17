#!/usr/bin/python
 
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
rospy.init_node('gripper_control')
 
# Create an action client
client = actionlib.SimpleActionClient(
    '/gripper/follow_joint_trajectory',  # namespace of the action topics
    GripperCommandAction # action type
)
    
# Wait until the action server has been started and is listening for goals
client.wait_for_server()
 
# Create a goal to send (to the action server)
goal = control_msgs.msg.GripperCommandGoal()
goal.command.position = 0.4   # From 0.0 to 0.8
goal.command.max_effort = -1.0  # Do not limit the effort
client.send_goal(goal)
 
client.wait_for_result()
