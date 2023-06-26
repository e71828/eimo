
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
        self.setpoint_yaw = 0
        self.base_output = 0
        self.pid = PID(1, 0.05, 0.2, setpoint=self.setpoint_yaw, error_map=pi_clip)
        self.pid.sample_time = 1 / rospy.get_param('/angle_frequency', default=10)
        self.pid.output_limits = (-100, 100)

        self.pi = pigpio.pi()
        if not self.pi.connected:
            return

        i2c_port = rospy.get_param('~i2c_port', default='/dev/i2c-3')

        self.pwm = PCA9685.PWM(self.pi, bus=int(i2c_port[-1]))  # defaults to bus 3, address 0x40
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
            if data.light1 and not data.up and not data.down:
                self.light1_level = self.light1_level + 1 if self.light1_level < 8 else 0
                self.pwm.set_pulse_width(6, self.light1_level*50 + 1100)
            if data.light2 and not data.up and not data.down:
                self.light2_level = self.light2_level + 1 if self.light2_level < 8 else 0
                self.pwm.set_pulse_width(7, self.light2_level*50 + 1100)

            self.base_output = 0
            if data.forward:
                self.base_output = -data.gain
                rospy.loginfo('withdraw')
            elif data.backward:
                self.base_output = data.gain
                rospy.loginfo('forward')
            elif data.turn_left:
                self.setpoint_yaw -= 5
                self.setpoint_yaw = pi_clip(self.setpoint_yaw)
                rospy.loginfo('turn left')
            elif data.turn_right:
                self.setpoint_yaw += 5
                self.setpoint_yaw = pi_clip(self.setpoint_yaw)
                rospy.loginfo('turn right')
        elif args == 2:
            self.current_yaw = data.yaw
            self.pid.setpoint = self.setpoint_yaw
            if abs(self.setpoint_yaw) == 180:
                self.pid.setpoint = abs(self.setpoint_yaw) if data.yaw > 0 else -abs(self.setpoint_yaw)
            output = self.pid(self.current_yaw)
            rospy.loginfo(f'setpoint_yaw: {self.setpoint_yaw}')
            rospy.loginfo(f'current  yaw: {self.current_yaw}')
            rospy.loginfo(f'output: {output}')
            rospy.loginfo(f'base_output: {self.base_output}')
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

if __name__ == "__main__":
    try:
        I2cPropel()
    except rospy.ROSInterruptException:
        pass
