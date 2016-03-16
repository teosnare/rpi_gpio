RPi-GPIO
========

A ROS node for managing GPIO pins on a Raspberry Pi

Disclaimer
----------

This code was developed and tested under ROS Indigo on Ubuntu 14.04. This is research code, and any fitness for a particular purpose is disclaimed.

Installation
------------

Initialize your project's custom [stacks directory](http://wiki.ros.org/ROS/Tutorials/StackInstallation) then checkout the git project like:

    cd ~/myproject/stacks
    git clone https://github.com/chrisspen/rpi_gpio.git

When initializing your shell environment, ensure your custom stacks directory is in the `ROS_PACKAGE_PATH` environment variable. A script to set your workspace environment to automatically do this might look like:

    . ./devel/setup.sh
    ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/myproject/stacks
    export ROS_PACKAGE_PATH

Usage
-----

Run with:

    rosrun rpi_gpio rpi_gpio_node.py

or:

    roslaunch rpi_gpio rpi_gpio.launch

See `config/example.yaml` for an illustration of parameters accepted.

Three basic parameters are available:

1. directions: A hash listing which pins are inputs (in) or outputs (out). Any pin not listed here will be assumed not in use and will not be managed by the node.
2. states: A hash listing the initial state of each output pin.
3. shutdown_states: A hash listing the state to set output pins to before safely shutting down.

Once running, you can set pins states from the command line with:

    rosservice call /rpi_gpio/set_pin 21 1
    rosservice call /rpi_gpio/set_pin 21 0

Or by calling the service from Python like:

    from rpi_gpio.srv import DigitalWrite
    rpi_gpio = rospy.ServiceProxy('/rpi_gpio/set_pin', DigitalWrite)
    pin = 21
    state = 1
    rpi_gpio(pin, state)
