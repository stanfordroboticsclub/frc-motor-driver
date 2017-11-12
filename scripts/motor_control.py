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

def clamp(value,mi,ma):
    return min(max(value,mi), ma)
    
def listener():

    rospy.init_node('motor_driver_pwm')

    rospy.Subscriber("turtle1/cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    bus.write_byte_data(address,128,128)
    listener()




