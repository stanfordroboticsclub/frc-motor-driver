#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

import RPi.GPIO as RPIO

# def setup():

    # global servo_left
    # global servo_right

left_servo_pin = 3
right_servo_pin = 4

RPIO.setmode(RPIO.BCM) 
RPIO.setup(left_servo_pin, RPIO.OUT)
RPIO.setup(right_servo_pin, RPIO.OUT)

servo_left = RPIO.PWM(left_servo_pin,50)
servo_right = RPIO.PWM(right_servo_pin,50)


servo_left.start(7.5)
servo_right.start(7.5)

def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    # data.linear
    # data.angular

    left = clamp(1500 + data.linear.x* 250 + (data.angular.z * 250),1400,1600)
    right = clamp(1500 + data.linear.x* 250 - (data.angular.z * 250),1400,1600)

    servo_left.ChangeDutyCycle(left * 50/10000)
    servo_right.ChangeDutyCycle(right* 50/10000)

def clamp(value,mi,ma):
    return min(max(value,mi), ma)
    
def listener():

    rospy.init_node('motor_driver_pwm')

    rospy.Subscriber("/turtle1/cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()




