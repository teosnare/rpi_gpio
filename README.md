RPi-GPIO
========

A ROS node for managing GPIO pins on a Raspberry Pi

Disclaimer
----------

This code was developed and tested under ROS Indigo on Ubuntu 14.04. This is research code, and any fitness for a particular purpose is disclaimed.

Installation
------------

todo

Usage
-----

Run with:

    rosrun rpi_gpio rpi_gpio_node.py

or:

    roslaunch rpi_gpio rpi_gpio.launch

See `config/example.yaml` for an illustration of parameters accepted.

Three basic parameters are available:

    1. directions: A hash listing which pins are inputs or outputs. Any pin not listed here will be assumed not in use and unmanaged by the node.
    2. states: A hash listing the initial state to set output pins.
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
