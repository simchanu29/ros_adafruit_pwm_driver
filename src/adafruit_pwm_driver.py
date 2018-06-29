#!/usr/bin/env python
# coding=utf-8

import rospy
from pwmboard_msgs.msg import PwmCmd
# from adafruit_pwm_drivers.Adafruit_PWM_Servo_Driver.Adafruit_PWM_Servo_Driver import PWM
from Adafruit_Python_PCA9685.Adafruit_PCA9685.PCA9685 as PWM

class PWMBoard:
    def __init__(self, address, actuators, sensors, data_types):

        # Gestion de la carte
        self.pwm = PWM(address)
        self.pwm.set_pwm_freq(50)

        # Formattage des données (quelle pin de la carte associée à quoi)
        self.devices_by_pins = self.gen_dic_by_pin_keys(actuators
        self.devices_by_name = actuators

        self.types = data_types
        # self.sensors = sensors
        print 'devices_by_pins : ', self.devices_by_pins

        # for device in self.devices_by_name:
            # pin = self.devices_by_name[device]['pin']
            # data_type = self.devices_by_name[device]['data_type']
            # self.setAccel(pin, self.types[data_type]['accel'])

    def gen_dic_by_pin_keys(self, devices):
        """
        Transforme la table de hachage où on accède aux numéros des pins par le nom de l'appareil en une table de
        hachage où on accède au nom de l'appareil par son numéro de pin associé
        :param pwm_devices:
        :return pin_dic:
        """
        pin_dic = dict()
        for device in devices:
            print 'device :', device
            pin = int(devices[device]['pin'])
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
        self.set_pwm(msg.pin, cmd)

    def set_pwm(self, pin, cmd):
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
        self.pwm.set_pwm(pin, 0, cmd)  # max 4095, 1 cycle = 4095. Donc 409 max (20ms -> 2ms)

    # def publish(self, sensors):
    #     for device in sensors:
    #         pub = sensors[device]['publisher']
    #         pin = int(sensors[device]['pin'])
    #
    #         # rospy.loginfo("getting positions")
    #         val = self.get_position(pin)
    #         # rospy.loginfo("Sensors values")
    #         pub.publish(val)


if __name__ == '__main__':

    rospy.init_node('adafruit_pwm_driver')

    rospy.loginfo("adafruit_pwm_driver Node Initialised")
    address = rospy.get_param('~address', 0x40)
    devices = rospy.get_param('adafruit_pwm/device')
    data_types = rospy.get_param('adafruit_pwm/data_types')

    actuators = {}
    sensors = {}
    for device in devices:
        print data_types[devices[device]['data_type']]['type']
        # if data_types[devices[device]['data_type']]['type']=='input':
        #     sensors[device] = devices[device]
        #     sensors[device]['publisher'] = rospy.Publisher(device, Float32, queue_size=1)
        if data_types[devices[device]['data_type']]['type']=='output':
            actuators[device] = devices[device]

    adafruit = PWMBoard(address, actuators, sensors, data_types)
    rospy.Subscriber('cmd_pwm', PwmCmd, adafruit.cb_pwm)

    while not rospy.is_shutdown():
        try:
            rospy.rostime.wallsleep(0.1)
            adafruit.publish(sensors)
        except rospy.ROSInterruptException:
            adafruit.close()
