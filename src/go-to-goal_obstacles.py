#! /usr/bin/env python

import rospy
import math
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import LinkState
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from PID_Control import PID_Pose_2D, PID_Control


class Navigation(object):
    def __init__(self):
        self._link_name = '/::base_link'
        self._reference_frame = ''

        self._topic_to_pub = '/cmd_vel'
        self._topic_to_sub = '/scan'
        
        self._vel_publisher = rospy.Publisher(self._topic_to_pub, 
                                    Twist, queue_size=1)
        self._laser_subscriber = rospy.Subscriber(self._topic_to_sub, 
                                    LaserScan, self.getLaser)

        self._robot_pose = LinkState().pose
        self._laser_scan = list()
        self._speed = Twist()

        self._rate = rospy.Rate(20)


    def getPose(self):
        try:
            link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
            link_pose = link_state(self._link_name, self._reference_frame)
            self._robot_pose = link_pose.link_state.pose
        except rospy.ServiceException as srv_exc:
            rospy.loginfo("Get Link State service call failed: {0}".format(srv_exc))


    def getLaser(self, laser_data):
        self._laser_scan = list(laser_data.ranges)
        

    
class Go_to_Goal(Navigation):
    def __init__(self, goal_x, goal_y):
        super(Go_to_Goal, self).__init__()
        self._goal_x = goal_x
        self._goal_y = goal_y

        self._pid_linear = PID_Pose_2D()
        self._pid_linear.set_KPID(1.0, 0.0, 0.0)
        self._pid_linear.setSetPoint(self._goal_x, self._goal_y)

        self._pid_angular = PID_Pose_2D()
        self._pid_angular.set_KPID(1.0, 0.0, 0.0)
        self._pid_angular.setSetPoint(self._goal_x, self._goal_y)

    ########### WALL FOLLOWER
        self._MOVING_FOWARD = 'moving_foward'
        self._FOLLOWING_RIGHT = 'following_right'
        self._FOLLOWING_LEFT = 'following_left'
        
        self._last_state = self._MOVING_FOWARD

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

    def get_around(self):
        max_distance = 30.0
        front_distance = max_distance
        right_distance = max_distance
        left_distance = max_distance

        l_distance_threshold = 2.0
        f_distance_threshold = 2.5 

        for value in self._laser_scan[330:390]:
            if value < front_distance:
                front_distance = value

        for value in self._laser_scan[40:330]:
            if value < left_distance:
                left_distance = value

        for value in self._laser_scan[390:680]:
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

################

    def move(self):
        self.getPose()
        accepted_error = 0.2

        orientation_list = [self._robot_pose.orientation.x,
                            self._robot_pose.orientation.y,
                            self._robot_pose.orientation.z,
                            self._robot_pose.orientation.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        
        sample = [self._robot_pose.position.x,
                  self._robot_pose.position.y,
                  yaw]

        self._pid_linear.addNewSample(sample)
        self._pid_linear.compute_errors()

        if self._pid_linear.linear_distance <= accepted_error:
            rospy.signal_shutdown('Shutdown')
            print self._robot_pose.position.x
            print self._robot_pose.position.y
        
        self._pid_angular.addNewSample(sample)
        self._pid_angular.compute_errors()

        angular_distance = self._pid_angular.angular_distance*180/math.pi

        if angular_distance >= 180:
            angular_distance -= 360
        if angular_distance > 135:
            angular_distance = 135.0
        if angular_distance < -135:
            angular_distance = -135.0

        laser_goal = int(360 - 2.67*angular_distance)

        wall_min_distance = 3.5
        
        for distance in self._laser_scan[laser_goal:laser_goal+1]:
            if distance < wall_min_distance:
                self.get_around()

            else:
                print 'GO'
                self._speed.linear.x = self._pid_linear.process_linear()
                self._speed.angular.z = self._pid_angular.process_angular()  

        self._vel_publisher.publish(self._speed)

    

if __name__ == '__main__':
    try:
        rospy.init_node('robot_navigation')
        goal_x = float(input('Goal X: '))
        goal_y = float(input('Goal Y: '))
        
        robot = Go_to_Goal(goal_x, goal_y)
        while not rospy.is_shutdown():
            robot.move()        

    except rospy.ROSInterruptException():
        pass
