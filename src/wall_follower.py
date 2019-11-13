#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from PID_Control import PID_Control


class Follower():
    def __init__(self):
        self._topic_to_pub = '/cmd_vel'
        self._topic_to_sub = '/scan'
        self._vel_publisher = rospy.Publisher(self._topic_to_pub, Twist, queue_size=1)
        self._laser_subscriber = rospy.Subscriber(self._topic_to_sub, LaserScan, self.get_laser)

        self._MOVING_FOWARD = 'moving_foward'
        self._FOLLOWING_RIGHT = 'following_right'
        self._FOLLOWING_LEFT = 'following_left'
        
        self._last_state = self._MOVING_FOWARD
        
        self._speed = Twist()
        self._laser_scan = tuple()

        self._rate = rospy.Rate(20)


    def get_laser(self, laser_data):
        self._laser_scan = laser_data.ranges


    def move_foward(self):
        print "Moving Foward"
        self._speed.linear.x = 0.8
        self._speed.angular.z = 0.0

        self._last_state = self._MOVING_FOWARD


    def turn_right(self):
        print "Turning right"
        self._speed.linear.x = 0.3
        self._speed.angular.z = -0.8
    

    def turn_left(self):
        print "Turning left"
        self._speed.linear.x = 0.3
        self._speed.angular.z = 0.8
    

    def follow_right(self, distance):
        print "Following right. Distance = ", distance
        pid_angular = PID_Control()
        pid_angular.set_KPID(2.0, 0.5, 0.1)
        pid_angular.setSetPoint(1.2)
        pid_angular.addNewSample(distance)
        vel_angular = pid_angular.process()
        
        self._last_state = self._FOLLOWING_RIGHT

        self._speed.linear.x = 0.6
        self._speed.angular.z = vel_angular


    def follow_left(self, distance):
        print "Following left. Distance = ", distance
        pid_angular = PID_Control()
        pid_angular.set_KPID(-2.0, -0.5, -0.1)
        pid_angular.setSetPoint(1.2)
        pid_angular.addNewSample(distance)
        vel_angular = pid_angular.process()

        self._last_state = self._FOLLOWING_LEFT

        self._speed.linear.x = 0.6
        self._speed.angular.z = vel_angular


    def move(self):
        max_distance = 30.0
        front_distance = max_distance
        right_distance = max_distance
        left_distance = max_distance

        l_distance_threshold = 2.0
        f_distance_threshold = 2.5 

        for value in self._laser_scan[330:390]:
            if value < front_distance:
                front_distance = value

        for value in self._laser_scan[60:330]:
            if value < left_distance:
                left_distance = value

        for value in self._laser_scan[390:660]:
            if value < right_distance:
                right_distance = value


        if front_distance > f_distance_threshold:
            if right_distance > l_distance_threshold and left_distance > l_distance_threshold:
                self.move_foward()
            elif right_distance > l_distance_threshold and left_distance < l_distance_threshold:
                self.follow_left(left_distance)
            else:
                self.follow_right(right_distance)
        else:
            if right_distance < l_distance_threshold and left_distance < l_distance_threshold:
                if self._last_state == self._FOLLOWING_RIGHT:
                    self.turn_left()
                else:
                    self.turn_right()
            elif right_distance < l_distance_threshold and left_distance > l_distance_threshold:
                self.turn_left()
            else:
                self.turn_right()
        
        self._vel_publisher.publish(self._speed)

        self._rate.sleep()



if __name__ == '__main__':
    try:
        rospy.init_node('wall_follower')   
        nav = Follower()    
        while not rospy.is_shutdown():
            nav.move()  

    except rospy.ROSInterruptException:
        pass
