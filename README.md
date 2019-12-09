# Package ROS for the Adafruit PWM Board

![Status](https://img.shields.io/badge/Status-In%20Development-red.svg)
![ROS](https://img.shields.io/badge/ROS-Kinetic--Kame-green.svg)
[![GitHub license](https://img.shields.io/github/license/simchanu29/ros_adafruit_pwm_driver.svg)](https://github.com/simchanu29/ros_adafruit_pwm_driver/blob/master/LICENSE)

### Usage

Used for [Adafruit 16-Channel PWM / Servo HAT for Raspberry Pi](https://www.adafruit.com/product/2327).

Listen for `PwmDeviceArray` messages on `/cmd_pwm_board` by default and send accordingly the commands to the board by 
I2C.

To test the package without any prior configurations, launch : 

```bash
roslaunch adafruit_pwm_driver test.launch
```

It will run the driver and the interpreter with the example configuration.

### Configuration

This package needs to be configured with a YAML config file. An example is in the config folder with and example launch 
file.

Example : 

```yaml
pwm_board_config:
# If your board doesn't handle inputs (sensors) then defining device with a data_type.type:'inputs' will make the
# node throw an error
  data_types:
    'motor100':
      type: 'output'
      range: [-100,100]
      accel: 10
    'attopilot':
      type: 'input'

  device:
    'thruster_left':
      'pin': 0
      'data_type': 'motor100'
    'battery1_voltage_raw':
      'pin': 8
      'data_type': 'attopilot'
```

### Nodes

 - adafruit_pwm_driver.py
    - Parameters

    ```
    pwm_board_config/device : list of devices connected to the board
    pwm_board_config/data_types : list of data types
    ```

    - Subscriptions

    ```
    cmd_pwm : PwmCmdArray
    ```

    - Publications

    ```

    ```
    
 - adafruit_cmd_interpreter.py
    - Parameters

    ```
    pwm_board_config/device : list of devices connected to the board
    ~sub_name : topic on which the node suscribe
    ```

    - Subscriptions

    ```
    sub_name : PwmDeviceArray (default : cmd_pwm_board)
    ```

    - Publications

    ```
    cmd_pwm : PwmCmdArray
    ```

### Notes

The command is in ms. For example 1000 is minimum, and 2000 is maximum pwm.



