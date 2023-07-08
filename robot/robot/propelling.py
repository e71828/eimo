
from .PCA9685 import PWM
import rclpy
from rclpy.node import Node
import pigpio
from time import sleep
from eimo_msgs.msg import Control, Angle
from simple_pid import PID


def pi_clip(angle):
    if angle > 180:
        return angle - 360
    elif angle < -180:
        return angle + 360
    else:
        return angle
# 175->180->-175->-180->175


class I2cPropel(Node):
    def __init__(self):
        super().__init__('propelling_control')
        self.pi = pigpio.pi()
        if not self.pi.connected:
            return
        self.declare_parameter('~i2c_port', '/dev/i2c-3')
        self.declare_parameter('init_yaw', 0)
        self.declare_parameter('angle_frequency', 10)

        self.i2c_port = self.get_parameter('~i2c_port').get_parameter_value().string_value
        self.pwm = PWM(self.pi, bus=int(self.i2c_port[-1]))  # defaults to bus 3, address 0x40
        self.pwm.set_frequency(50)  # suitable for servos
        self.pwm.set_pulse_width(4, 1500)  # -1 for all channels
        self.pwm.set_pulse_width(5, 1500)  # -1 for all channels
        sleep(3)  # init the speed controller

        self.light1_level = -1
        self.light2_level = -1
        self.yaw = []
        self.current_yaw = .0
        self.current_yaw_v = None
        self.current_yaw_a = None
        self.setpoint_yaw = self.get_parameter('init_yaw').get_parameter_value().integer_value

        self.base_output = 0

        self.pid = PID(1 ,0.05, 0.2, setpoint=self.setpoint_yaw, error_map=pi_clip)
        self.frequency = self.get_parameter('depth_frequency').get_parameter_value().integer_value
        self.pid.sample_time = 1 / self.frequency
        self.pid.output_limits = (-100, 100)

        self.sub_control = self.create_subscription(Control, 'control', self.controlling, 1)
        self.sub_angle = self.create_subscription(Angle, 'angle', self.controlling, 2)

    def __del__(self):
        sleep(3)  # wait for other using
        self.pwm.cancel()
        self.pi.stop()
        self.sub_control.destroy()
        self.sub_angle.destroy()
        self.gget_logger().info('Welcome Back to Stop Safely')

    def controlling(self, data, args):
        if args == 1:
            if data.light1 and not data.up and not data.down:
                self.light1_level = self.light1_level + 1 if self.light1_level < 8 else 0
                self.pwm.set_pulse_width(6, self.light1_level*50 + 1100)
            if data.light2 and not data.up and not data.down:
                self.light2_level = self.light2_level + 1 if self.light2_level < 8 else 0
                self.pwm.set_pulse_width(7, self.light2_level*50 + 1100)

            self.base_output = 0
            if data.forward:
                self.base_output = -data.gain
                self.get_logger().info('withdraw')
            elif data.backward:
                self.base_output = data.gain
                self.get_logger().info('forward')
            elif data.turn_left:
                self.setpoint_yaw -= 5
                self.setpoint_yaw = pi_clip(self.setpoint_yaw)
                self.get_logger().info('turn left')
            elif data.turn_right:
                self.setpoint_yaw += 5
                self.setpoint_yaw = pi_clip(self.setpoint_yaw)
                self.get_logger().info('turn right')
        elif args == 2:
            self.current_yaw = data.yaw
            self.pid.setpoint = self.setpoint_yaw
            output = self.pid(self.current_yaw)
            self.get_logger().info(f'setpoint_yaw: {self.setpoint_yaw}')
            self.get_logger().info(f'current  yaw: {self.current_yaw}')
            self.get_logger().info(f'output: {output:.1f}')
            self.get_logger().info(f'base_output: {self.base_output:.1f}')
            if self.base_output > 0:
                self.pwm.set_pulse_width(4, 1524  - output + self.base_output)
                self.pwm.set_pulse_width(5, 1527  + output + self.base_output)
            elif self.base_output < 0:
                self.pwm.set_pulse_width(4, 1453  - output + self.base_output)
                self.pwm.set_pulse_width(5, 1453  + output + self.base_output)
            elif output > 2:
                self.pwm.set_pulse_width(4, 1453  - output)
                self.pwm.set_pulse_width(5, 1527  + output)
            elif output < -2:
                self.pwm.set_pulse_width(4, 1524  - output)
                self.pwm.set_pulse_width(5, 1453  + output)
            else:
                self.pwm.set_pulse_width(4, 1500)
                self.pwm.set_pulse_width(5, 1500)


def main(arg=None):
    rclpy.init(args=arg)
    yaw_control_node = I2cPropel()
    rclpy.spin(yaw_control_node)
    yaw_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
