#!/usr/bin/env python

import rospy
import numpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Navigation():
    
    def __init__(self):

        self._subLaser = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self._pubCmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._laser = []

    def laser_callback(self, msg):
        
        self._laser = list(msg.ranges)
           
    def min_dist(self):
        
        speed = Twist()

        for x in self._laser:
            print(x)
            if x <= 1.5:
                
                #Linear velocity
                speed.linear.x = 0.4

                #Angular velocity
                speed.angular.z = -1

                self._pubCmdVel.publish(speed)
                

            else:

                #Linear velocity
                speed.linear.x = 0.8

                self._pubCmdVel.publish(speed)

if __name__ == "__main__":
    rospy.init_node("Navigation", anonymous=True)
    nav = Navigation()

    while not rospy.is_shutdown():

        nav.min_dist()