
from pca9685 import PCA9685
import rospy
import pigpio
import time
from eimo_msgs.msg import control
from eimo_msgs.msg import angle
from simple_pid import PID


def pi_clip(angle):
    if angle > 0:
        if angle > 180:
            return angle - 360
    else:
        if angle < -180:
            return angle + 2 * 360
    return angle


class I2cPropel:
    def __init__(self):
        rospy.init_node('propelling_control', anonymous=True)
        self.yaw = []
        self.current_yaw = .0
        self.current_yaw_v = None
        self.current_yaw_a = None
        self.setpoint_yaw = .0
        self.controlling_flag = False
        self.base_output = 0
        self.pid = PID(0.5, 0.05, 0.05, setpoint=self.setpoint_yaw, error_map=pi_clip)
        self.pid.sample_time = 0.1  # Update every 0.1 seconds
        self.pid.output_limits = (-20, 20)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            return

        i2c_port = rospy.get_param('~i2c_port', default='/dev/i2c-3')

        self.pwm = PCA9685.PWM(self.pi, bus=int(i2c_port[-1]))  # defaults to bus 1, address 0x40
        self.pwm.set_frequency(50)  # suitable for servos
        self.pwm.set_pulse_width(4, 1500)  # -1 for all channels
        self.pwm.set_pulse_width(5, 1500)  # -1 for all channels
        time.sleep(3)

        self.light1_level = -1
        self.light2_level = -1

        rospy.on_shutdown(self.shutdown)
        self.sub_control = rospy.Subscriber('control', control, self.controlling, 1)
        self.sub_angle = rospy.Subscriber('angle', angle, self.controlling, 2)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        pass

    def shutdown(self):
        self.pwm.cancel()
        self.pi.stop()
        self.sub_control.unregister()
        self.sub_angle.unregister()
        rospy.loginfo('Welcome Back to Stop Safely')

    def controlling(self, data, args):
        if args == 1:
            if data.light1:
                self.light1_level = self.light1_level + 1 if self.light1_level < 8 else 0
                self.pwm.set_pulse_width(6, self.light1_level*50 + 1100)
            if data.light2:
                self.light2_level = self.light2_level + 1 if self.light2_level < 8 else 0
                self.pwm.set_pulse_width(7, self.light2_level*50 + 1100)

            if not data.turn_left and not data.turn_right:
                if data.forward:
                    self.controlling_flag = True
                    self.base_output = data.gain / 256 * 4
                    self.pwm.set_pulse_width(4, 1527 + self.base_output)
                    self.pwm.set_pulse_width(5, 1520 + self.base_output)
                    rospy.loginfo('forward')
                elif data.backward:
                    self.controlling_flag = True
                    self.base_output = data.gain / 256 * (-10)
                    self.pwm.set_pulse_width(4, 1454 + self.base_output)
                    self.pwm.set_pulse_width(5, 1454 + self.base_output)
                    rospy.loginfo('withdraw')
                elif self.controlling_flag:
                    self.pwm.set_pulse_width(4, 1500)
                    self.pwm.set_pulse_width(5, 1500)
                    self.controlling_flag = False
                else:
                    pass
            elif data.turn_left:
                self.controlling_flag = True
                self.pwm.set_pulse_width(4, 1527)
                self.pwm.set_pulse_width(5, 1454)
                rospy.loginfo('turn left')
            elif data.turn_right:
                self.controlling_flag = True
                self.pwm.set_pulse_width(4, 1527)
                self.pwm.set_pulse_width(5, 1454)
                rospy.loginfo('turn right')
            else:
                pass
        elif args == 2:
            self.current_yaw = data.yaw
            # self.yaw.insert(0, self.current_yaw)
            # self.yaw.pop() if len(self.yaw) > 10 else None

            # self.current_yaw_v = data.yaw_v
            # self.current_yaw_a = data.yaw_a
            if not self.controlling_flag:
                self.pid.setpoint = self.setpoint_yaw
                output = self.pid(self.current_yaw)
                # rospy.loginfo(f'current output: {output}')
                # self.pwm.set_pulse_width(4, 1527  + output)  # TODO
                # self.pwm.set_pulse_width(5, 1454  - output)
            else:
                self.setpoint_yaw = data.yaw


if __name__ == "__main__":
    try:
        I2cPropel()
    except rospy.ROSInterruptException:
        pass
