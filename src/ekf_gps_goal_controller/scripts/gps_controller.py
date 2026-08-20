#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import tf
# import utm

class GPSController:
    def __init__(self):
        rospy.init_node('gps_goal_controller')


        rospy.Subscriber('/imu/data', Imu, self.imu_callback) # lấy yaw từ IMU
        
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.loginfo("GPS Goal Controller started...")
    

    # ---------------------------- LẤY YAW TỪ IMU ----------------------------
    def imu_callback(self, msg):
        """Lấy yaw trực tiếp từ IMU"""
        orientation = msg.orientation
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(q)

        self.current_yaw = yaw

        rospy.loginfo(f"Yaw: {math.degrees(self.current_yaw):.1f} deg")



if __name__ == '__main__':
    try:
        GPSController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass