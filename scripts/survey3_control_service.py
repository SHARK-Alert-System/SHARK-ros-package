#!/usr/bin/env python

from __future__ import print_function

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String
from uav_pi.srv import Survey3, Survey3Response
import time


GPIO.setwarnings(False)

pin = 18  #set (BCM) GPIO pin to send GPIO.HIGH pulse
key = "n"

response = Survey3Response()
response.isMounted = True

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.OUT)
time.sleep(0.001)
GPIO.cleanup()

def handle_pwm_control(req):
    if req.command == "t":
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.002)
        GPIO.cleanup()
        
        time.sleep(0.1)
        rospy.loginfo("survey3_ctrl_service: Survey3 Photo Triggered via PWM")        
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        time.sleep(0.001)
        GPIO.cleanup()
    elif req.command == "s":
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.0015)
        GPIO.cleanup()
        
        time.sleep(0.1)

        response.isMounted = bool(not response.isMounted)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        time.sleep(0.001)
        GPIO.cleanup()      
    else:        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        time.sleep(0.001)
        GPIO.cleanup()

    ans = "survey3_ctrl_service: Survey3 is Unmounted"
    if response.isMounted:
        ans = "survey3_ctrl_service: Survey3 is Mounted"
    rospy.loginfo(ans)
    return response

def object_detect_server():
    rospy.init_node('survey3_control_service')
    s = rospy.Service('survey3_command', Survey3, handle_pwm_control)
    rospy.loginfo("survey3_ctrl_service: Node started. Ready to control survey3.")
    rospy.spin()

if __name__ == "__main__":
    object_detect_server()
