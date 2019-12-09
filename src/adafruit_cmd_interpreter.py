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
from std_msgs.msg import Float64


class PWMInterpreter:

    def __init__(self, devices, device_types):
        self.devices = devices
        self.device_types = device_types

        for d in self.devices:
            if float(self.device_types[self.devices[d]['type']]['max']) < float(self.device_types[self.devices[d]['type']]['max']):
                rospy.logerr("Max command < Min command in parameters. Exit preventively.")
                exit()

    def cb_interpreter(self, msg):
        """
        Reçoit un tableau de PwmDevice et pour chacun d'entre eux extrait la pin associée t renvoie le message.
        :param msg: PwmDeviceArray
        :return:  type: 'output'
          range: [-50,50]
          accel: 10

        """
        msg_to_send = PwmCmdArray()

        for msgcmd in msg.array:
            msg_pwm = PwmCmd()
            msg_pwm.command = self.cmd_to_pwm(msgcmd.command, msgcmd.device)
            msg_pwm.pin = self.device_to_pin(msgcmd.device, self.devices)
            msg_to_send.array.append(msg_pwm)

        pub.publish(msg_to_send)

    def cmd_to_pwm(self, cmd, device_name):
            print('cmd :', cmd)  # commande en int
            print('device_name', device_name)

            # Gestion du type de commande
            device_type = self.devices[device_name]['type']
            print('device_type :', device_type)
            range_cmd = self.device_types[device_type]['range']
            print('range_cmd=', range_cmd, range_cmd[0])
            range_min = float(range_cmd[0])
            range_max = float(range_cmd[1])

            range_tot = range_max-range_min
            range_zero = range_min + range_tot/2.0
            print('range=', range_cmd)

            max_cmd = float(self.device_types[device_type]['max'])
            min_cmd = float(self.device_types[device_type]['min'])

            deadband = self.device_types[device_type]['deadband']
            deadband_min = float(deadband[0])
            deadband_max = float(deadband[1])

            deadband_offset = self.device_types[device_type]['deadband_offset']
            deadband_offset_min = float(deadband_offset[0])
            deadband_offset_max = float(deadband_offset[1])

            print("deadband={}".format(deadband))
            print("deadband_offset={}".format(deadband_offset))
            print("min={} max={}".format(min_cmd, max_cmd))

            a_max = (max_cmd - deadband_offset_max)/(max_cmd - deadband_max)
            a_min = (min_cmd - deadband_offset_min)/(min_cmd - deadband_min)
            b_max = deadband_offset_max - a_max * deadband_max
            b_min = deadband_offset_min - a_min * deadband_min

            # Calcul de la commande
            cmd = max(min(cmd, max_cmd), min_cmd)
            print("cmd max/min {}".format(cmd))
            if cmd <= deadband_max and cmd >= deadband_min:
                cmd = 0.0
            elif cmd < deadband_min:
                cmd = cmd * a_min + b_min
            elif cmd > deadband_max:
                cmd = cmd * a_max + b_max

            print("cmd % {}".format(cmd))

            # Calcul de la commande en pwm
            pwm_cmd = (cmd - range_zero)*1000/range_tot + 1500 # cmd to pwm
            print("pwm_cmd = {}".format(pwm_cmd))

            return pwm_cmd

    def device_to_pin(self, device_name, devices_dic):
        return int(devices_dic[device_name]['pin'])

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node("cmd_interpreter")

    rospy.loginfo('Import config')
    rospy.loginfo(rospy.get_param_names())


    # Common configuration for any PWM Board
    devices = rospy.get_param('pwm_board_config/device')
    device_types = rospy.get_param('pwm_board_config/data_types')
    actuators = {}
    sensors = {}
    for device in devices:
        print device_types[devices[device]['type']]['type']
        if device_types[devices[device]['type']]['type']=='input':
            sensors[device] = devices[device]
            sensors[device]['publisher'] = rospy.Publisher(device, Float64, queue_size=1)
        if device_types[devices[device]['type']]['type']=='output':
            actuators[device] = devices[device]
    # Sensor data is here ignored since adafruit pwm board doesn't handle inputs

    sub_name = rospy.get_param('~sub_name', 'cmd_pwm_board')

    pwm_interpreter = PWMInterpreter(actuators, device_types)

    sub = rospy.Subscriber(sub_name, PwmDeviceArray, pwm_interpreter.cb_interpreter)
    pub = rospy.Publisher('cmd_pwm', PwmCmdArray, queue_size=1)

    pwm_interpreter.spin()
