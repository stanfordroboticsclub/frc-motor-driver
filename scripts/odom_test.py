#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import tf

from threading import Timer
import math



class Controller:

    def __init__(self):
        self.wheel_base = 0.3 # placeholder value
        self.world_frame = 'odom'
        self.robot_frame = 'base_link'

        self.odom_pub = rospy.Publisher(self.world_frame, Odometry, queue_size=10)
        self.odom_trans = tf2_ros.TransformBroadcaster()

        self.x = 0
        self.y = 0
        self.th = 0

        self.vx = 0
        self.vy = 0
        self.vth = 0

        self.time = rospy.Time.now()

    def calculate_offset(self,left,right):

        new_time = rospy.Time.now() 
        interval = (new_time - self.time).to_sec()
        self.time = new_time

        dth = (left - right)/self.wheel_base

        alpha = (math.pi - dth)/2 - self.th
        length = math.sqrt( 2* ( right/dth + self.wheel_base/2)**2 * ( 1- math.cos(alpha) ) )

        dx = length * math.cos(alpha)
        dy = length * math.sin(alpha)

        #calculation may not be correct as velocities are in local frame?
        self.vx = dx / interval
        self.vy = dy / interval
        self.vth = dth / interval

        self.x += dx
        self.y += dy
        self.th += dth

    def read_encoders(self):

        left_dist  = 0.12
        right_dist = 0.08

        self.calculate_offset(left_dist,right_dist)

        trans = self.generate_odom_trans()
        self.odom_trans.sendTransform(trans);

        message = self.generate_odom_message()
        self.odom_pub.publish(message)

        encoder_timer = Timer(0.1, self.read_encoders, ())
        encoder_timer.start()

    def generate_odom_trans(self):
        #self.odom_quat = tf.createQuaternionMsgFromYaw(self.th)

        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_quat = Quaternion(*q)

        odom_transform = TransformStamped()
        odom_transform.header.stamp = self.time
        odom_transform.header.frame_id = self.world_frame
        odom_transform.child_frame_id = self.robot_frame

        odom_transform.transform.translation.x = self.x
        odom_transform.transform.translation.y = self.y
        odom_transform.transform.translation.z = 0.0
        odom_transform.transform.rotation = self.odom_quat
        return odom_transform

    def generate_odom_message(self):
        odom = Odometry()
        odom.header.stamp = self.time 
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.robot_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self.odom_quat

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth

        return odom


    
def listener():

    rospy.init_node('motor_driver_pwm')

    control = Controller()
    control.read_encoders()

    rospy.spin()

if __name__ == '__main__':
    listener()




