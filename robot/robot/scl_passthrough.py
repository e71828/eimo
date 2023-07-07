#!/bin/python3
"""
scl_passthrough.py

Used to connect to a motor driver, and forward/received and ros messages.
"""

import rclpy
from rclpy.node import Node
from eimo_msgs.srv import Scl
import serial
from time import sleep


class SCLPassthroughNode(Node):
    def __init__(self):
        super().__init__('scl_passthrough')
        self.declare_parameter('~serial_port', '/dev/ttyAMA0')
        self.declare_parameter('~baudrate', 9600)
        self.declare_parameter('~parity', serial.PARITY_NONE)
        self.declare_parameter('~stop_bits', serial.STOPBITS_ONE)

        self.rx = self.create_service(Scl, 'scl_passthrough', self.tx_transmit)

        serial_port = self.get_parameter('~serial_port').get_parameter_value().string_value
        baudrate = self.get_parameter('~baudrate').get_parameter_value().integer_value

        try:
            self.serial_port = serial.Serial(serial_port, baudrate=baudrate, timeout=0)
        except serial.SerialException as e:
            self.get_logger().error("Unable to open serial port {}".format(e))
            return

    def tx_transmit(self, req, res):
        data = req.request
        self.get_logger().debug("Transmitting {}".format(data))

        try:
            packet = data.encode('ascii') + b'\r'
            self.serial_port.write(packet)
        except serial.SerialException as e:
            self.get_logger().warning("Unable to write to serial")

        sleep(0.03)

        try:
            data = self.serial_port.readline()
        except serial.SerialException:
            self.get_logger().debug("Error reading from serial")

        if data:
            self.get_logger().debug("Receiving {}".format(data))
            res.answer = data.decode('utf8', errors='ignore')
            return res
        else:
            res.answer = ''
            return res

def main(args=None):
    rclpy.init(args=args)
    scl_node = SCLPassthroughNode()
    rclpy.spin(scl_node)
    scl_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()