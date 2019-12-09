## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['Adafruit_PCA9685', 'Adafruit_GPIO'],
    package_dir={'': 'include',
                 'Adafruit_PCA9685': 'Adafruit_Python_PCA9685/Adafruit_PCA9685',
                 'Adafruit_GPIO': 'Adafruit_Python_GPIO/Adafruit_GPIO'})

setup(**setup_args)

# install PCA9685
# import Adafruit_Python_PCA9685
# setup('install')
