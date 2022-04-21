#! /usr/bin/env python3
import rospy
import time

from adafruit_servokit import ServoKit
from std_msgs.msg import Int32

## initialize the servo kit instance
kit = ServoKit(channels=16)

## callback function for the rear motor
def callback(data):    
    kit.servo[4].angle = data.data

## callback function for the front motor
def callback2(data):
    kit.servo[11].angle = data.data

## callback function for the servo motor
def callback3(data):
    print(data.data)
    kit.servo[15].angle = data.data

def start():
    kit = ServoKit(channels=16)

    ## initalize the esc of the motor
    # kit.servo[4].angle = 0
    # kit.servo[11].angle = 0
    # kit.servo[15].angle = 90
    # time.sleep(2)
    # kit.servo[15].angle = 45
    # kit.servo[15]._pwm
    # kit.servo[2]._pwm = 32768
    for i in range (16):
        kit.continuous_servo[i].set_pulse_width_range(500,2000)        
        kit.continuous_servo[i].throttle = 1
        # kit.servo[i].set_pulse_width_range(500,2000)
        # for angle in range(0,180):
        #     kit.servo[i].angle = angle
        # kit.servo[i].angle = 45     

    # rospy.Subscriber("rear_motor", Int32, callback)
    # rospy.Subscriber("front_motor",Int32, callback2)
    # rospy.Subscriber("servo_motor",Int32, callback3)

    # starts the node
    # rospy.init_node('motor')
    # rospy.spin()

if __name__ == '__main__':
    start()
