#!/usr/bin/env python
"""
serial_passthrough.py

Used to connect to an arm, and forward and ros messages received and
"""

import rospy
from eimo_msgs.srv import scl, sclResponse
import serial
from time import sleep


class SCLPassthroughNode:

    def __init__(self):

        self.rx = rospy.Service('scl_passthrough', scl, self.tx_transmit)
        rospy.init_node('serial_passthrough', anonymous=True, log_level=rospy.DEBUG)

        serial_port = rospy.get_param('~serial_port', default='/dev/ttyAMA0')
        baudrate = rospy.get_param('~baudrate', default=9600)
        parity = rospy.get_param('~parity', default=serial.PARITY_NONE)
        stop_bits = rospy.get_param('~stop_bits', default=serial.STOPBITS_ONE)

        try:
            self.serial_port = serial.Serial(serial_port, baudrate=baudrate, parity=parity,
                                             stopbits=stop_bits, timeout=0)
            rospy.loginfo("Opened serial port {}".format(serial_port))
        except serial.SerialException as e:
            rospy.logerr("Unable to open serial port {}".format(e))
            return

        rospy.spin()

    def tx_transmit(self, req):
        data = req.Request
        rospy.logdebug("Transmitting {}".format(data))

        # encoded_packet = WITProtocol.encode_packet(device_id, packet_id, data)
        try:
            packet = data.encode('ascii') + b'\r'
            self.serial_port.write(packet)
        except serial.SerialException as e:
            rospy.logwarn("Unable to write to serial")

        sleep(0.1)

        try:
            data = self.serial_port.readline()
        except serial.SerialException:
            rospy.logdebug("Error reading from serial")

        if data:
            rospy.logdebug("Receiving {}".format(data))
            return sclResponse(data.decode('utf8'))
        else:
            return sclResponse('')


if __name__ == '__main__':
    passthrough_node = SCLPassthroughNode()
