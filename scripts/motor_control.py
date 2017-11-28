#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import tf

import smbus
from threading import Timer

bus = smbus.SMBus(1)

address = 0x04
curent_timer = None

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard stuff")

    # data.linear
    # data.angular


    left = clamp(128 + data.linear.x* 64 + (data.angular.z * 64),100,156)
    right = clamp(128 + data.linear.x* 64 - (data.angular.z * 64),100,156)

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

def read_encoders():
    try:
        data = bus.read_i2c_block_data(address,0)
    except IOError:
        print 'IOERROR suppressing'
        encoder_timer = Timer(0.1, read_encoders, ())
        encoder_timer.start()
        return

    left_dist  =((data[0] << 24) + (data[1] << 16) + (data[2] << 8) + data[3])/10000
    right_dist =((data[4] << 24) + (data[5] << 16) + (data[6] << 8) + data[7])/10000

    time = rospy.Time.now() 

    odom_quat = tf.createQuaternionMsgFromYaw(th);

    odom_trans = TransformStamped()
    odom_trans.header.stamp = time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_trans.sendTransform(odom_trans);

    odom = Odometry()
    odom.header.stamp = time 
    odom.header.frame_id = 'odom'

    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom_pub.publish(odom)


    encoder_timer = Timer(0.1, read_encoders, ())
    encoder_timer.start()


def clamp(value,mi,ma):
    return min(max(value,mi), ma)
    
def listener():
    global odom_pub, odom_trans
    stop()

    rospy.init_node('motor_driver_pwm')
    rospy.Subscriber("turtle1/cmd_vel", Twist, callback)
    odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)

    odom_trans = tf.TransformBroadcaster()

    read_encoders()
    rospy.spin()

if __name__ == '__main__':
    listener()




