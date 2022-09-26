from pca9685 import PCA9685
import rospy
import pigpio
from eimo_msgs.msg import control
from eimo_msgs.msg import angle


class I2cPropel:
    def __init__(self):
        rospy.init_node('propelling_control', anonymous=True)
        rospy.Subscriber('control', control, self.controlling, 1)
        rospy.Subscriber('angle', angle, self.controlling, 2)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            return

        i2c_port = rospy.get_param('~i2c_port', default='/dev/i2c-3')

        self.pwm = PCA9685.PWM(self.pi, bus=int(i2c_port[-1]))  # defaults to bus 1, address 0x40
        self.pwm.set_frequency(50)  # suitable for servos

        self.light1_brightness_level = 0
        self.light2_brightness_level = 0

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        pass

    def __del__(self):
        self.pwm.cancel()
        self.pi.stop()

    def controlling(self, cmd_data, args):
        rospy.logdebug(rospy.get_caller_id() + "I heard cmd: go forward %s", cmd_data.forward)

        if cmd_data.light1:
            self.light1_brightness_level = self.light1_brightness_level + 1 if self.light1_brightness_level < 5 else 0
            self.pwm.set_duty_cycle(6, self.light1_brightness_level + 6)
        if cmd_data.light2:
            self.light2_brightness_level = self.light2_brightness_level + 1 if self.light2_brightness_level < 5 else 0
            self.pwm.set_duty_cycle(7, self.light2_brightness_level + 6)

        if not cmd_data.turn_left and not cmd_data.turn_right:
            if cmd_data.forward:
                base_value = cmd_data.gain / 256 * 1
            elif cmd_data.backward:
                base_value = cmd_data.gain / 256 * (-1)
            else:
                base_value = 0
            self.keep_yaw(base_value, angle.yaw, angle.yaw_v, angle.yaw_a)
        elif cmd_data.turn_left:
            self.pwm.set_duty_cycle(4, 7.7)
            self.pwm.set_duty_cycle(5, 7.3)
        elif cmd_data.turn_right:
            self.pwm.set_duty_cycle(4, 7.3)
            self.pwm.set_duty_cycle(5, 7.7)
        else:
            pass

    def keep_yaw(self, step_value, angle, angle_v, angle_a):
        pass  # TODO


if __name__ == "__main__":
    try:
        I2cPropel()
    except rospy.ROSInterruptException:
        pass
