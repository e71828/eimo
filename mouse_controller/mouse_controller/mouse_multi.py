# !/bin/python3
import rclpy
from rclpy.node import Node
import pyspacemouse
from time import sleep
from eimo_msgs.msg import Control


# cast int args to bool tuple (e.g. (1, 0, 1) -> (True, False, True))
def i2b(*args):
    return tuple(map(bool, args))


class MouseMulti(Node):
    def __init__(self):
        super().__init__('publish_control_cmd')
        self.publisher_ = self.create_publisher(Control, 'Control', 10)
        print("Devices found:\n\t%s" % "\n\t".join(pyspacemouse.list_devices()))
        try:
            dev = pyspacemouse.open()
            print(dev.describe_connection())
        except Exception as e:
            print("Unable to connect to device.")
            return
        state = pyspacemouse.read()
        if state:
            print(
                " ".join(
                    [
                        "%4s %+.2f" % (k, getattr(state, k))
                        for k in ["x", "y", "z", "roll", "pitch", "yaw", "t"]
                    ]
                )
            )
            print("".join(["buttons=", str(state.buttons)]))

        sleep(4)  # wait here
        self.declare_parameter('~frequency', 1)
        self.frequency = self.get_parameter('~frequency').get_parameter_value().integer_value
        self.threshold = 0.9
        self.gain = 64
        self.rec_num = int(100 / self.frequency)
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

    def timer_callback(self):
        forward, backward, right, left, up, down, button_0, button_1 = False, False, False, False, False, False, False, False
        for _ in range(self.rec_num):
            state = pyspacemouse.read()
            if state.y > self.threshold:
                forward = True
            elif state.y < -self.threshold:
                backward = True
            if state.yaw > self.threshold:
                right = True
            elif state.yaw < -self.threshold:
                left = True
            if state.z > self.threshold:
                up = True
            elif state.z < -self.threshold:
                down = True
            if state.buttons[0]:
                button_0 = True
            if state.buttons[1]:
                button_1 = True
            sleep(0.3 / self.rec_num)

        cmd = Control()
        cmd.gain = self.gain
        if right and not left:
            cmd.forward, cmd.backward, cmd.up, cmd.down, cmd.turn_left, cmd.turn_right, cmd.light1, cmd.light2 = i2b(0, 0, 0, 0, 0, 1, button_0, button_1)
        elif left and not right:
            cmd.forward, cmd.backward, cmd.up, cmd.down, cmd.turn_left, cmd.turn_right, cmd.light1, cmd.light2 = i2b(0, 0, 0, 0, 1, 0, button_0, button_1)
        elif up and not down and not button_0 and not button_1:
            cmd.forward, cmd.backward, cmd.up, cmd.down, cmd.turn_left, cmd.turn_right, cmd.light1, cmd.light2 = i2b(0, 0, 1, 0, 0, 0, 0, 0)
        elif down and not up and not button_0 and not button_1:
            cmd.forward, cmd.backward, cmd.up, cmd.down, cmd.turn_left, cmd.turn_right, cmd.light1, cmd.light2 = i2b(0, 0, 0, 1, 0, 0, 0, 0)
        elif up and not down and button_0 and button_1:
            cmd.forward, cmd.backward, cmd.up, cmd.down, cmd.turn_left, cmd.turn_right, cmd.light1, cmd.light2 = i2b(0, 0, 1, 0, 0, 0, 1, 1)
        elif down and not up and button_0 and button_1:
            cmd.forward, cmd.backward, cmd.up, cmd.down, cmd.turn_left, cmd.turn_right, cmd.light1, cmd.light2 = i2b(0, 0, 0, 1, 0, 0, 1, 1)
        elif forward and not backward:
            cmd.forward, cmd.backward, cmd.up, cmd.down, cmd.turn_left, cmd.turn_right, cmd.light1, cmd.light2 = i2b(1, 0, 0, 0, 0, 0, button_0, button_1)
        elif backward and not forward:
            cmd.forward, cmd.backward, cmd.up, cmd.down, cmd.turn_left, cmd.turn_right, cmd.light1, cmd.light2 = i2b(0, 1, 0, 0, 0, 0, button_0, button_1)
        elif button_0 or button_1:
            cmd.forward, cmd.backward, cmd.up, cmd.down, cmd.turn_left, cmd.turn_right, cmd.light1, cmd.light2 = i2b(0, 0, 0, 0, 0, 0, button_0, button_1)
        else:
            cmd.forward, cmd.backward, cmd.up, cmd.down, cmd.turn_left, cmd.turn_right, cmd.light1, cmd.light2 = i2b(0, 0, 0, 0, 0, 0, 0, 0)
        self.publisher_.publish(cmd)


def main(arg=None):
    rclpy.init(args=arg)
    mouse_multi = MouseMulti()
    rclpy.spin(mouse_multi)
    mouse_multi.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()