#!/usr/bin/env python
# coding=utf-8

import rospy
from ros_bprime_drivers.msg import PwmCmd
from adafruit_pwm_drivers.Adafruit_PWM_Servo_Driver.Adafruit_PWM_Servo_Driver import PWM


class PWMBoard:
    def __init__(self, pwm_devices, command_types):

        # Gestion de la carte
        self.pwm = PWM(0x40)
        self.pwm.setPWMFreq(50)

        # Formattage des données (quelle pin de la carte associée à quoi)
        self.pins = self.gen_dic_by_pin_keys(pwm_devices)
        self.devices = pwm_devices
        self.types = command_types
        print 'pins : ', self.pins

    def gen_dic_by_pin_keys(self, pwm_devices):
        """
        Transforme la table de hachage où on accède aux numéros des pins par le nom de l'appareil en une table de
        hachage où on accède au nom de l'appareil par son numéro de pin associé
        :param pwm_devices:
        :return pin_dic:
        """
        pin_dic = dict()
        for device in pwm_devices:
            print 'device :', device
            pin = int(pwm_devices[device]['pin'])
            pin_dic[pin] = device
        return pin_dic

    def cb_pwm(self, msg):
        # TESTS, pour une frequence de 50Hz, et 0 en ON, on a des valeurs correcte pour les servo de 101 a 560
        # 16*6 = 96, 16*35=560
        # Milieu a 330 (230 de chaque cote)

        print 'pin :', msg.pin  # pin en int
        print 'cmd :', msg.command  # commande en int

        # Gestion du type de commande
        device_name = self.pins[msg.pin]
        print 'device_name', device_name
        type = self.devices[device_name]['command_type']
        print 'type', type
        range = self.types[type]['range']
        range_min = range[0]
        range_max = range[1]
        range_tot = range_max-range_min
        range_zero = range_min + range_tot/2.0
        print 'range', range

        # Calcul de la commande en pwm
        cmd = (msg.command - range_zero)*1000/range_tot + 1500

        # Envoi de la commande
        self.setPWM(msg.pin, cmd)

    def setPWM(self, pin, cmd):
        """
        Reçoit une commande en pwm et le traduit à l'échelle de la carte
        :param pin: int
        :param cmd: float
        :return:
        """
        print 'pin :', pin
        print 'pwm cmd :', cmd
        cmd = int((cmd - 1000) / 1000 * 409)
        print 'setPWM cmd', cmd
        self.pwm.setPWM(pin, 0, cmd)  # max 4095, 1 cycle = 4095. Donc 409 max (20ms -> 2ms)


if __name__ == '__main__':
    rospy.init_node("motorBoard_driver")

    pwmboard = PWMBoard(rospy.get_param('pwm_device'), rospy.get_param('command_type'))
    rospy.Subscriber('pwm_cmd', PwmCmd, pwmboard.cb_pwm)

    rospy.spin()
