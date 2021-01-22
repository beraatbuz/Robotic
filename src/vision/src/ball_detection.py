#!/usr/bin/env python

import sys, time
import numpy as np
import imutils
import cv2
import roslib
import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from robot_control.msg import RobotVision
import roslaunch
import rosnode
import math

res_width = 320
res_height = 240
foallength_pixels = res_width / (2 * math.tan(math.radians(62.2) / 2));

a = "Z"
VERBOSE = True
redUpper1 = (140, 255, 255)
redLower1 = (100, 150, 0)
redUpper2 = (110, 255, 255)
redLower2 = (101, 50, 38)
startNode = True


class image_feature:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''

        self.info_pub = rospy.Publisher("/robot_vision",
                                        RobotVision, queue_size=1)

        self.image_sub = rospy.Subscriber("/rrbot/camera1/image_raw/compressed",
                                          CompressedImage, self.callback_image, queue_size=1)

        if VERBOSE:
            print "subscribed to /rrbot/camera1/image_raw"

    def callback_image(self, ros_data):
        '''Callback function of subscribed topic.
        Here images get converted and features detected'''
        if VERBOSE:
            print 'received image of type: "%s"' % ros_data.format

        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        global a
        global redUpper1
        global redUpper2
        global redLower1
        global redLower2
        print(a[0])
        nodes = rosnode.get_node_names()
        found = False
        for i in nodes:
            if i == '/control_node':
                found = True

        if not found:
            a = "Z"
            while a[0] != 'R' and a[0] != 'B' and a[0] != 'Y':
                a = (raw_input("R or B or Y: "))
                if (a[0] == 'R'):
                    print("here loop red")
                    redUpper1 = (10, 255, 255)
                    redLower1 = (0, 70, 50)
                    redUpper2 = (180, 255, 255)
                    redLower2 = (170, 70, 50)
                    node = roslaunch.core.Node(package="control", node_type="control_node",
                                               name="control_node", args='1')
                    launch = roslaunch.scriptapi.ROSLaunch()
                    launch.start()
                    script = launch.launch(node)

                elif (a[0] == 'Y'):
                    print("here loop yellow")
                    redUpper1 = (30, 255, 255)
                    redLower1 = (20, 100, 100)
                    redUpper2 = (40, 150, 255)
                    redLower2 = (23, 41, 133)
                    node = roslaunch.core.Node(package="control", node_type="control_node",
                                               name="control_node", args='3')
                    launch = roslaunch.scriptapi.ROSLaunch()
                    launch.start()
                    script = launch.launch(node)
                else:
                    node = roslaunch.core.Node(package="control", node_type="control_node",
                                               name="control_node", args='2')
                    launch = roslaunch.scriptapi.ROSLaunch()
                    launch.start()
                    script = launch.launch(node)
        if a[0] == 'R':
            print("HERE red")
            redUpper1 = (10, 255, 255)
            redLower1 = (0, 70, 50)
            redUpper2 = (180, 255, 255)
            redLower2 = (170, 70, 50)
        elif a[0] == 'Y':
            print("HERE yellow")
            redUpper1 = (30, 255, 255)
            redLower1 = (20, 100, 100)
            redUpper2 = (40, 150, 255)
            redLower2 = (23, 41, 133)
        elif a[0] == 'B':
            print("HERE BLUE")
            redUpper1 = (140, 255, 255)
            redLower1 = (100, 150, 0)
            redUpper2 = (110, 255, 255)
            redLower2 = (101, 50, 38)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, redLower1, redUpper1)
        mask2 = cv2.inRange(hsv, redLower2, redUpper2)
        mask = mask1 | mask2
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow('mask', mask)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None
        control_info = RobotVision()
        control_info.Ball = False
        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            dist = -1
            if radius > 5:
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                dist = (foallength_pixels * (10)) / (radius)

            control_info.BallCenterX = center[0]
            control_info.BallCenterY = center[1]
            control_info.Ball = 1
            control_info.BallRadius = np.uint8(int(radius))
            control_info.DistBall = dist * (0.01)
            cv2.imshow('window', image_np)
            cv2.waitKey(2)
            control_info.Ball = True
        self.info_pub.publish(control_info)


def main(args):
    '''Initializes and cleanup ros node'''
    global a
    print(a[0])
    print(rosnode.get_node_names())

    while a[0] != 'R' and a[0] != 'B' and a[0] != 'Y':
        a = (raw_input("R or B or Y: "))
        if (a[0] == 'R'):
            print("here loop red")
            redUpper1 = (10, 255, 255)
            redLower1 = (0, 70, 50)
            redUpper2 = (180, 255, 255)
            redLower2 = (170, 70, 50)
            node = roslaunch.core.Node(package="control", node_type="control_node",
                                       name="control_node", args='1')
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()
            script = launch.launch(node)

        elif (a[0] == 'Y'):
            print("here loop yellow")
            redUpper1 = (30, 255, 255)
            redLower1 = (20, 100, 100)
            redUpper2 = (40, 150, 255)
            redLower2 = (23, 41, 133)
            node = roslaunch.core.Node(package="control", node_type="control_node",
                                       name="control_node", args='3')
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()
            script = launch.launch(node)
        else:
            node = roslaunch.core.Node(package="control", node_type="control_node",
                                       name="control_node", args='2')
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()
            script = launch.launch(node)

    rospy.init_node('ball_detection', anonymous=True)
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
