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

DEBUG = False

import rospy
from pwmboard_msgs.msg import PwmCmdArray
from std_msgs.msg import Float32
if not DEBUG:
    from Adafruit_PCA9685.PCA9685 import PCA9685 as PWM


class PWMBoard:
    def __init__(self, address):

        # Gestion de la carte
        if not DEBUG:
            self.pwm = PWM(address)
            self.pwm.set_pwm_freq(50)

        self.gain_pwm_to_cmd = rospy.get_param('~gain_pwm_to_cmd', 0.225) # Valeurs déterminées expérimentalements
        self.offset_pwm_to_cmd = rospy.get_param('~offset_pwm_to_cmd', 0.525) # Valeurs déterminées expérimentalements

    def cb_pwm(self, msg):
        """
        Reçoit un tableau de commande et le traite
        :param msg: PwmCmdArray
        :return:
        """
        for msgcmd in msg.array:

            # Envoi de la commande
            self.set_pwm(msgcmd.pin, msgcmd.command)

    def set_pwm(self, pin, cmd):
        """
        Reçoit une commande en pwm et le traduit à l'échelle de la carte
        :param pin: int
        :param cmd: float
        :return:
        """
        print 'pin :', pin
        print 'pwm cmd :', cmd
        # cmd = int((cmd - 1000) / 1000 * 409)
        cmd = int(self.gain_pwm_to_cmd*cmd + self.offset_pwm_to_cmd)
        print 'setPWM cmd', cmd

        if not DEBUG:
            self.pwm.set_pwm(pin, 0, cmd)  # max 4095, 1 cycle = 4095. Donc 409 max (20ms -> 2ms)


if __name__ == '__main__':

    # ROS init
    rospy.init_node('pwm_driver')
    rospy.loginfo("pwm_driver Node Initialised")

    # Node init
    address = int(rospy.get_param('~address', "0x40"), 16)

    # Launch PWM Board
    pwm_board = PWMBoard(address)
    rospy.Subscriber('cmd_pwm', PwmCmdArray, pwm_board.cb_pwm)

    # Spin while ROS is running
    rospy.spin()
