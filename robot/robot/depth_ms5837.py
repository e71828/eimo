#!/bin/python3
import rclpy
from rclpy.node import Node
import ms5837
from time import sleep
from eimo_msgs.msg import Depth


class I2cMs5837(Node):
    def __init__(self):
        super().__init__('publish_depth')
        self.depth_publisher = self.create_publisher(Depth, 'depth', 10)
        self.declare_parameter('depth_frequency', 1)
        self.declare_parameter('~i2c_port', '/dev/i2c-1')
        self.declare_parameter('~density', 897)
        self.declare_parameter('init_depth', 0)

        self.frequency = self.get_parameter('depth_frequency').get_parameter_value().integer_value
        self.i2c_port = self.get_parameter('~i2c_port').get_parameter_value().string_value
        self.sensor = ms5837.MS5837_02BA(int(self.i2c_port[-1]))  # Default I2C bus is 1
        self.fluid_density = self.get_parameter('~density').get_parameter_value().integer_value
        self.sensor.setFluidDensity(self.fluid_density) # kg/m^3

        # We must initialize the sensor before reading it
        if not self.sensor.init():
            self.get_logger().error("Depth sensor could not be initialized from i2c port {}".format(self.i2c_port))
            return
        else:
            self.get_logger().info("Opened i2c port {}".format(self.i2c_port))
            self.sensor.read(ms5837.OSR_8192)
            depth_value = 0
            for i in  range(6):
                depth_value += self.sensor.depth()
            self.init_depth = depth_value / 6
            self.get_logger().info('init m: {}'.format(self.init_depth))

            init_depth = rclpy.parameter.Parameter(
                'init_depth',
                rclpy.Parameter.Type.INTEGER,
                int(self.init_depth * 1000)
            )
            self.set_parameters([init_depth])

            self.timer = self.create_timer(1 / self.frequency, self.pub_depth)

    def pub_depth(self):
        depth_value = 0
        # read depth 90 times and get the average value
        for _ in range(30):
            self.sensor.read(ms5837.OSR_512)
            depth_value += self.sensor.depth()
            sleep(0.01)
        depth_value /= 30
        depth_value_mm = int(depth_value * 1000 + 300)
        if depth_value_mm < 0:
            depth_value_mm = 0
        msg = Depth()
        msg.depth_mm = depth_value_mm
        self.depth_publisher.publish(msg)
        self.get_logger().info('mm: {}'.format(depth_value_mm))


def main(arg=None):
    rclpy.init(args=arg)
    depth_node = I2cMs5837()
    rclpy.spin(depth_node)
    depth_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()