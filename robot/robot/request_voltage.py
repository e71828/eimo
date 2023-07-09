#!/bin/python3

"""request_voltage.py"""

import rclpy
from eimo_msgs.srv import Scl


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('request_voltage')
    cli = node.create_client(Scl, 'scl_passthrough')
    node.get_logger().debug('Hello to debug')

    voltage_req = Scl.Request()
    voltage_req.request = 'IU'
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    future = cli.call_async(voltage_req)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    vol_str = result.answer

    if len(vol_str) > 2 and 'IU=' in vol_str:
        index = vol_str.index('IU=')
        voltage = int(vol_str[index + 3:index + 7], 16) / 10
        node.get_logger().debug('voltage is %.1f V' % voltage)
        if voltage <= 22.2:
            node.get_logger().warning('voltage is low: %.1f V' % voltage)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()