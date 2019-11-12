#!/usr/bin/env python

import cv2
import rospy
import camera
from geometry_msgs.msg import Point
from cv2 import aruco


class Tag_ident():
    def __init__(self):

        self.start = camera.Camera()
        self.pub_goal = rospy.Publisher("/Warthog/goal", Point, queue_size=10)

    def ident(self):

        gray = cv2.cvtColor(self.start.get(), cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        self.frame_markers = aruco.drawDetectedMarkers(self.start.get(), corners, ids)
        goal = Point()
        pic = False
        if ids is not None:
            if ids[0] == 0:
                rospy.loginfo("Aruco id = 0 found!")
                goal.x = 4
                goal.y = 8
                self.pub_goal.publish(goal)

            elif ids[0] == 1:
                rospy.loginfo("Aruco id = 1 found!")
                if not pic:
                    rospy.sleep(.5)
                    cv2.imwrite("img/aruco_1.png", self.start.get())
                    rospy.sleep(.5)
                    pic = True
                goal.x = 0
                goal.y = 8
                self.pub_goal.publish(goal)

            elif ids[0] == 4:
                rospy.loginfo("Aruco id = 4 found!")
                goal.x = 0
                goal.y = 0
                self.pub_goal.publish(goal)    

    def show_img(self):

        cv2.imshow("Tag ident", self.frame_markers)
        cv2.waitKey(1)


if __name__ == "__main__":

    rospy.init_node("Tag_ident_node", anonymous=True)
    tag_ident = Tag_ident()

    while not rospy.is_shutdown():
        tag_ident.ident()
        tag_ident.show_img()
    cv2.destroyAllWindows()
