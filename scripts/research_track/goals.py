#!/usr/bin/env python
# license removed for brevity

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():
    goals = [{'x':-0.3049999475479126,'y':0.11999991536140442},{'x': 1.5167715549468994,'y': -2.4407434463500977},{'x': 1.5500670671463013,'y': 0.08256609737873077},{'x': 4.085905075073242,
    'y': -0.10053956508636475},{'x':0.3049999475479126,'y':0.-11999991536140442}]
    for idx, goal1 in enumerate(goals):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal1['x']
        goal.target_pose.pose.orientation.w = goal1['y']
        client.send_goal(goal)
        wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
