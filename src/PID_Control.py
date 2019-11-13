import rospy
import math

class PID_Control():
    def __init__(self):
        self._kp = 0.0
        self._ki = 0.0
        self._kd = 0.0
        self._P = 0.0
        self._I = 0.0
        self._D = 0.0
        self._PID = 0.0
        self._sample = 0.0
        self._set_point = 0.0
        self._last_sample = 0.0
        self._last_process = 0.0

    def set_KPID(self, kp, ki, kd):
        self._kp = kp
        self._ki = ki   
        self._kd = kd

    def setSetPoint(self, set_point):
        self._set_point = set_point
    
    def addNewSample(self, sample):
        self._sample = sample

    def process(self):
        error = self._set_point - self._sample
        dt = (rospy.get_time() - self._last_process)/1000
        self._last_process = rospy.get_time()

        self._P  = self._kp*error
        self._I += self._ki*error*dt
        self._D  = self._kd*(self._last_sample - self._sample)*dt

        self._last_sample = self._sample

        self._PID = self._P + self._I + self._D
        return self._PID


class PID_Pose_2D():
    def __init__(self):
        self._kp = self._ki = self._kd = 0.0
        self._goal_x = self._goal_y = 0.0
        self._x = self._y = self._yaw = 0.0
        self.linear_distance = self.angular_distance = 0.0
        self._last_process = self._last_linear = self._last_angular = 0.0
        self._P = self._I = self._D = self._PID = 0.0

    def set_KPID(self, kp, ki, kd):
        self._kp = kp
        self._ki = ki
        self._kd = kd


    def setSetPoint(self, goal_x, goal_y):
        self._goal_x = goal_x
        self._goal_y = goal_y


    def addNewSample(self, sample):
        self._x = sample[0]
        self._y = sample[1]
        self._yaw = sample[2]

    
    def compute_errors(self):
        error_x = self._goal_x - self._x
        error_y = self._goal_y - self._y
        self.linear_distance = math.sqrt(error_x**2 + error_y**2)

        desired_theta = math.atan2(error_y, error_x)
        self.angular_distance = desired_theta - self._yaw
        

    def process_linear(self):
        dt = (rospy.get_time() - self._last_process)/1000
        self._last_process = rospy.get_time()

        self._P  = self._kp*self.linear_distance
        self._I += self._ki*self.linear_distance*dt
        self._D  = self._kd*(self._last_linear - self.linear_distance)*dt

        self._last_linear = self.linear_distance

        self._PID = self._P + self._I + self._D
        return self._PID
        
  
    def process_angular(self):
        dt = (rospy.get_time() - self._last_process)/1000
        self._last_process = rospy.get_time()

        self._P  = self._kp*self.angular_distance
        self._I += self._ki*self.angular_distance*dt
        self._D  = self._kd*(self._last_angular - self.angular_distance)*dt

        self._last_angular = self.angular_distance

        self._PID = self._P + self._I + self._D

        return self._PID
