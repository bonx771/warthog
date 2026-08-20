#!/usr/bin/env python3
# import roslib
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
import apriltag

class move_with_tag:

    def __init__(self):
        self.image_pub = rospy.Publisher("apriltag_detect", Image, queue_size=10)
        self.pub = rospy.Publisher("warthog_velocity_controller/cmd_vel", Twist, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("realsense/color/image_raw", Image, self.move)

    def move(self, data):
        f_x = 337.2084410968044
        f_y = 337.2084410968044
        c_x = 320.5
        c_y = 240.5

        intrinsic = np.array([
        [f_x, 0.0, c_x],
        [0.0, f_y, c_y],
        [0.0, 0.0, 1.0]
        ])

        distortion = np.array([1e-08, 1e-08, 1e-08, 1e-08, 1e-08])

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        h,  w = cv_image.shape[:2]

        # --------- Calibrate ---------
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(intrinsic,distortion,(w,h),1,(w,h))
        cv_image = cv2.undistort(cv_image, intrinsic, distortion, None, newcameramtx)

        # --------- Detect AprilTag ---------
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        options = apriltag.DetectorOptions(families="tag36h11")
        detector = apriltag.Detector(options)
        results = detector.detect(gray)
        for r in results:
            # extract the bounding box (x, y)-coordinates for the AprilTag
            # and convert each of the (x, y)-coordinate pairs to integers
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

            # draw the bounding box of the AprilTag detection
            cv2.line(cv_image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(cv_image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(cv_image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(cv_image, ptD, ptA, (0, 255, 0), 2)

            # draw the center (x, y)-coordinates of the AprilTag
            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(cv_image, (cX, cY), 5, (0, 0, 255), -1)

            # draw the tag family on the image
            tagFamily = r.tag_family.decode("utf-8")
            cv2.putText(cv_image, tagFamily, (ptA[0], ptA[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # --------- Move ---------
        # rate = rospy.Rate(10)
        # while not rospy.is_shutdown():
        cmd = Twist()
        angular = 0 #* math.atan2(tran[1], tran[0]) #angular vel with calibration of 3
        linear = 0 #* K #linear vel with calibration of 0.1
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.pub.publish(cmd)
        # rospy.loginfo("POS:({},{},{})".format(tran[0],tran[1],tran[2]))
        # rospy.loginfo("VEL:x={},theta={}".format(linear,angular))

        # rate.sleep()

        # --------- Publish Image ---------
        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
          print(e)

if __name__ == '__main__':
    rospy.init_node('warthog_move')
    mwt = move_with_tag()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting down...")