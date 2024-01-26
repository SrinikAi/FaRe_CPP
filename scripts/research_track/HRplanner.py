#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

class RobotController:
    def __init__(self):
        self.fire_detected = False
        self.battery_level = 100
        self.state = "idle"
        self.cancel_goal_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
     
    def move_to_goal(self, x, y):
        client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.orientation.w= y
        client.send_goal(goal)
        
        print(f"moving to goal: ({x},{y})")
        
        reached_goal = True
        
        if reached_goal:
           self.state = "idle"
        
        
    
    def stop_robot(self):
        cancel_msg = GoalID()
        self.cancel_goal_pub.publish(cancel_msg)
        print("Stopping the robot by canceling the goal.")
    
    def fire_detection_callback(self, msg):
        new_fire_detected = (msg.data == 1)
        if new_fire_detected != self.fire_detected:
            self.fire_detected = new_fire_detected
            self.reactive_behavior()
        #else:
          #self.hierarchial_planning()

    def charging_callback(self, msg):
        new_battery_level = msg.data
        if new_battery_level != self.battery_level:
            self.battery_level = new_battery_level
            self.reactive_behavior()
        #else:
          #self.hierarchial_planning()

    def reactive_behavior(self):
        if self.fire_detected:
            self.handle_fire_detection()
        elif self.battery_level < 10:
            self.handle_low_battery() 
    
    
    def hierarchial_planning(self):
    	 self.switch_to_exploration_mode()

    
    
    def handle_fire_detection(self):
        print("Fire detected! Activating fire extinguishing mode")
        if self.state == "idle":
          if self.battery_level < 10:
            self.handle_low_battery()
          else:
            self.execute_fire_extinguishing()
    
          

    def handle_low_battery(self):
        print("Battery level below 10! Activating charging mode")
        self.execute_charging()

    def execute_fire_extinguishing(self):
        
        self.state = "moving_to_goal"
        self.move_to_goal(0.3049999475479126,0.-11999991536140442)

    def execute_charging(self):
       
        self.state = "moving_to_goal"
        self.move_to_goal(1.5500670671463013,0.08256609737873077)
        
    def switch_to_exploration_mode(self):
        print ("robot is idle switching to exploration mode")
        if self.state == "idle":
           self.explore()
          
    def explore(self):
        goals =  [{'x':-0.3049999475479126,'y':0.11999991536140442},{'x': 1.5167715549468994,'y': -2.4407434463500977},{'x': 1.5500670671463013,'y': 0.08256609737873077},{'x': 4.085905075073242,
    'y': -0.10053956508636475},{'x':0.3049999475479126,'y':0.-11999991536140442}]
        for idx, goal1 in enumerate(goals):
            self.reactive_behavior()
            self.state = "moving_to_goal"
            self.move_to_goal(goal1['x'],goal1['y'])
           
        

def main():
    rospy.init_node('robot_controller', anonymous=True)
    controller = RobotController()

    rospy.Subscriber('/fire', Int32, controller.fire_detection_callback)
    rospy.Subscriber('/battery', Int32, controller.charging_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

