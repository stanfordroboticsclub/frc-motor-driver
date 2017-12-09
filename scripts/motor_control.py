#!/usr/bin/env python
from __future__ import division

import rospy
import tf2_ros
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import tf

import smbus
from threading import Timer
import math

bus = smbus.SMBus(1)

address = 0x04
curent_timer = None

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard stuff")

    left = clamp(128 + data.linear.x* 64 - (data.angular.z * 64),100,156)
    right = clamp(128 + data.linear.x* 64 + (data.angular.z * 64),100,156)

    try:
        bus.write_byte_data(address,int(left),int(right))    
    except IOError:
        print 'IOERROR suppressing'

    global curent_timer
    if curent_timer != None:
        curent_timer.cancel()

    curent_timer = Timer(1, stop, ())
    curent_timer.start()

def stop():
    try:
        bus.write_byte_data(address,128,128)    
    except IOError:
        print 'IOERROR suppressing'

class Controller:

    def __init__(self):
        self.wheel_base = 0.46
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

        self.last_left = 0
        self.last_right = 0

        self.time = rospy.Time.now()

    def calculate_offset(self,left,right):

        dleft = left - self.last_left
        dright = right - self.last_right
        self.last_left = left
        self.last_right = right



        new_time = rospy.Time.now() 
        interval = (new_time - self.time).to_sec()
        self.time = new_time

        dth = (dleft - dright)/self.wheel_base

        # aprox code
        d_center = (dleft + dright)/2
        dx = d_center * math.cos(-self.th)
        dy = d_center * math.sin(-self.th)


	rospy.loginfo('d '+str(dleft)+', '+str(dright)+', '+str(d_center))
	rospy.loginfo('c '+str(self.x)+', '+str(self.y)+', '+str(self.th))

        self.th = self.th % (2*math.pi)

        # alpha = (math.pi - dth)/2 - self.th

	# if dth == 0:
		# length = dleft
	# elif dleft+dright > 0:
		# length = math.sqrt( 2* ( dright/dth + self.wheel_base/2)**2 * ( 1- math.cos(dth) ) )

	# elif dleft+dright < 0:
		# length = -math.sqrt( 2* ( dright/dth + self.wheel_base/2)**2 * ( 1- math.cos(dth) ) )
	# else:
		# length = 0

        # dx = length * math.cos(alpha)
        # dy = length * math.sin(alpha)

	rospy.loginfo(str(dx)+', '+str(dy)+", "+str(dth))

	#local frame velocities
        self.vx = 0
        self.vy = (dleft+dright)/float(2) / interval
        self.vth = dth / interval

        self.x += dx
        self.y += dy
        self.th += dth
	#rospy.loginfo('c'+str(self.x)+', '+str(self.y)+", "+str(self.th))


    def decode_data(self,data):
        num = ((data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3])
        # implement two's complement
        if num & (1 << 31):
            num = ~abs(num) + 1
        return num/10000

    def read_encoders(self):
        try:
            data = bus.read_i2c_block_data(address,0)
        except IOError:
            print 'IOERROR suppressing'
            encoder_timer = Timer(0.1, self.read_encoders, ())
            encoder_timer.start()
            return

        left_dist  = self.decode_data(data[0:4])
        right_dist  = self.decode_data(data[4:8])

        self.calculate_offset(left_dist,right_dist)

        trans = self.generate_odom_trans()
        self.odom_trans.sendTransform(trans);

        message = self.generate_odom_message()
        self.odom_pub.publish(message)

        encoder_timer = Timer(0.1, self.read_encoders, ())
        encoder_timer.start()

    def generate_odom_trans(self):
        # not sure why the minus is here
        q = tf.transformations.quaternion_from_euler(0, 0, -self.th)
        #q = tf.transformations.quaternion_from_euler(0, 0, self.th)
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


def clamp(value,mi,ma):
    return min(max(value,mi), ma)
    
def listener():
    stop()

    rospy.init_node('motor_driver_pwm')
    rospy.Subscriber("turtle1/cmd_vel", Twist, callback)

    control = Controller()
    control.read_encoders()

    rospy.spin()

if __name__ == '__main__':
    listener()




