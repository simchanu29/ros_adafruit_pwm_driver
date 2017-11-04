#!/usr/bin/env python
# coding=utf-8

import rospy
from ros_bprime_drivers.msg import PwmCmd
from std_msgs.msg import Int16


def cb_interpreter(msg):
    msg_pwm = PwmCmd()
    msg_pwm.command = msg.data
    msg_pwm.pin = pin
    pub.publish(msg_pwm)

if __name__ == "__main__":
    rospy.init_node("adafruit_cmd_interpreter")

    devices = rospy.get_param('/pwm_device')
    device_name = rospy.get_param('~device_name')
    pin = int(devices[device_name]['pin'])
    print 'pin : ', pin

    sub_name = rospy.get_param('~sub_name')

    sub = rospy.Subscriber(sub_name, Int16, cb_interpreter)
    pub = rospy.Publisher('/pwm_cmd', PwmCmd, queue_size=1)
    rospy.spin()
