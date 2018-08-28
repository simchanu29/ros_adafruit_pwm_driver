#!/usr/bin/env python
# coding=utf-8

"""
Copyright 2018 Simon CHANU

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import rospy
from pwmboard_msgs.msg import PwmDeviceArray, PwmCmd, PwmCmdArray


def cb_interpreter(msg):
    """
    Reçoit un tableau de PwmDevice et pour chacun d'entre eux extrait la pin associée t renvoie le message.
    :param msg: PwmDeviceArray
    :return:
    """
    msg_to_send = PwmCmdArray()
    for msgcmd in msg.array:
        msg_pwm = PwmCmd()
        msg_pwm.command = msgcmd.data
        msg_pwm.pin = device_to_pin(msgcmd.device, devices)
        msg_to_send.array.append(msg_pwm)
    pub.publish(msg_to_send)


def device_to_pin(device_name, devices_dic):
    return int(devices_dic[device_name]['pin'])


if __name__ == "__main__":
    rospy.init_node("cmd_interpreter")

    # Common configuration for any PWM Board
    devices = rospy.get_param('pwm_board_config/device')
    sub_name = rospy.get_param('~sub_name', 'cmd_pwm_board')

    sub = rospy.Subscriber(sub_name, PwmDeviceArray, cb_interpreter)
    pub = rospy.Publisher('cmd_pwm', PwmCmdArray, queue_size=1)
    rospy.spin()
