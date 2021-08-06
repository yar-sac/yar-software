#!/bin/sh
arduino-cli compile --fqbn Seeeduino:samd:seeed_XIAO_m0 avionics
arduino-cli upload -p /dev/ttyACM0 --fqbn Seeeduino:samd:seeed_XIAO_m0 avionics