# Package ROS for the Adafruit PWM Board

### Usage
Send PwmMsg to the adafruit_pwm_driver.py node

```
PwmMsg
Int16 pin
Int16 command
```

The command is in ms. For example 1000 is minimum, and 2000 is maximum pwm.

This package needs to be configured with a YAML config file. An example is in the config folder with and example launch file.
