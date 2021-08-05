#!/bin/sh
arduino-cli compile --fqbn Seeeduino:samd:seeed_XIAO_m0 avionics
arduino-cli upload -p serial:///dev/cu.usbmodem14401 --fqbn Seeeduino:samd:seeed_XIAO_m0 avionics