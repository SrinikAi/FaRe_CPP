#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32 

class DecisionMaker:
    def __init__(self):
        self.fire_detected = False
        self.battery_level = 100
    
    def fire_detection_callback(self, msg):
        self.fire_detected = (msg.data == 1)
        self.make_decision()

    def charging_callback(self, msg):
        self.battery_level = msg.data
        self.make_decision()

    def make_decision(self):
        if self.fire_detected:
            print("Fire detected!")
            # Decide whether to prioritize fire or battery
            if self.battery_level < 10:
                self.handle_battery_priority()
            else:
                self.handle_fire_priority()
        elif self.battery_level < 10:
           
            self.handle_battery_priority()
        
    def handle_fire_priority(self):
        print("Switching to fire extinguishing mode...")
        # Implement fire extinguishing behaviors

    def handle_battery_priority(self):
        print("Battery level below 10! Switching to charging mode...")
        # Implement charging behaviors

def main():
    rospy.init_node('decision_node', anonymous=True)
    decision_maker = DecisionMaker()

    rospy.Subscriber('/fire', Int32, decision_maker.fire_detection_callback)
    rospy.Subscriber('/battery', Int32, decision_maker.charging_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
