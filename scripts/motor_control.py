#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

# import RPi.GPIO as RPIO
# from RPIO import PWM
import smbus
from threading import Timer

bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard stuff")

    # data.linear
    # data.angular


    left = clamp(128 + data.linear.x* 64 + (data.angular.z * 64),100,156)
    right = clamp(128 + data.linear.x* 64 - (data.angular.z * 64),100,156)

    bus.write_byte_data(address,int(left),int(right))    

    Timer(1, lambda : bus.write_byte_data(address,128,128), ()).start()

def clamp(value,mi,ma):
    return min(max(value,mi), ma)
    
def listener():

    rospy.init_node('motor_driver_pwm')

    rospy.Subscriber("turtle1/cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()




