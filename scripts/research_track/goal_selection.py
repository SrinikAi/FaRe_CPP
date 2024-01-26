#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import random
from nav_msgs.msg import OccupancyGrid
selected_areas = [(-0.3049999475479126,0.11999991536140442),( 1.5167715549468994, -2.4407434463500977),(1.5500670671463013,0.08256609737873077),(4.085905075073242,
    -0.10053956508636475),(0.3049999475479126,0.-11999991536140442)]
    
selected_goal_frequency = 3  # The robot will visi

MIN_X = 0.0
MAX_X = 0.0
MIN_Y = 0.0
MAX_Y = 0.0

def map_callback(msg):
    global MIN_X, MAX_X, MIN_Y, MAX_Y
    width = msg.info.width
    height = msg.info.height
    resolution = msg.info.resolution

    MIN_X = -width / 2 * resolution
    MAX_X = width / 2 * resolution
    MIN_Y = -height / 2 * resolution
    MAX_Y = height / 2 * resolution

def send_goal(x, y):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    client.wait_for_result()

def frontier_based_exploration():
    # Implement your frontier-based exploration algorithm here
    # For simplicity, let's just move the robot randomly within the map boundaries
    x, y = random.uniform(MIN_X, MAX_X), random.uniform(MIN_Y, MAX_Y)
    send_goal(x, y)

def explore_map():
    rospy.init_node('task_driven_exploration_node')
    rospy.Subscriber('/map', OccupancyGrid, map_callback)

    try:
        # Exploration loop
        while not rospy.is_shutdown():
            # Randomly choose whether to visit a selected goal or explore the map
            if random.randint(1, selected_goal_frequency) == 1:
                # Choose a selected goal randomly and revisit it
                x, y = random.choice(selected_areas)
                rospy.loginfo(f"Robot revisiting selected goal: ({x}, {y})")
                send_goal(x, y)
            else:
                # Explore the map using frontier-based exploration within the map boundaries
                frontier_based_exploration()

    except rospy.ROSInterruptException:
        rospy.loginfo("Task-driven exploration interrupted.")

if __name__ == '__main__':
    explore_map()
