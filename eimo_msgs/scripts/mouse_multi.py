
import pyspacemouse
import rospy
from rosgraph import is_master_online
from time import sleep
from eimo_msgs.msg import control


def send_control_cmd():
    print("Devices found:\n\t%s" % "\n\t".join(pyspacemouse.list_devices()))
    dev = pyspacemouse.open()
    if dev:
        print(dev.describe_connection())
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
        pub = rospy.Publisher('control', control, queue_size=1)
        rospy.init_node('publish_control_command', anonymous=True)
        frequency = rospy.get_param('~frequency', default=1)
        rate = rospy.Rate(frequency)  # 1hz
        threshold = 0.9
        gain = 64
        while not rospy.is_shutdown():
            forward, backward, right, left, up, down, button_0, button_1 = 0, 0, 0, 0, 0, 0, 0, 0
            for _ in range(90):
                state = pyspacemouse.read()
                if state.y > threshold:
                    forward = True
                elif state.y < -threshold:
                    backward = True
                if state.yaw > threshold:
                    right = True
                elif state.yaw < -threshold:
                    left = True
                if state.z > threshold:
                    up = True
                elif state.z < -threshold:
                    down = True
                if state.buttons[0]:
                    button_0 = True
                if state.buttons[1]:
                    button_1 = True
                sleep(0.01)

            if right and not left:
                control_cmd = control(0, 0, gain, 0, 0, 0, 1, button_0, button_1)
            elif left and not right:
                control_cmd = control(0, 0, gain, 0, 0, 1, 0, button_0, button_1)
            elif up and not down and not button_0 and not button_1:
                control_cmd = control(0, 0, gain, 1, 0, 0, 0, 0, 0)
            elif down and not up and not button_0 and not button_1:
                control_cmd = control(0, 0, gain, 0, 1, 0, 0, 0, 0)
            elif up and not down and button_0 and  button_1:
                control_cmd = control(0, 0, gain, 1, 0, 0, 0, 1, 1)
            elif down and not up and button_0 and  button_1:
                control_cmd = control(0, 0, gain, 0, 1, 0, 0, 1, 1)
            elif forward and not backward:
                control_cmd = control(1, 0, gain, 0, 0, 0, 0, button_0, button_1)
            elif backward and not forward:
                control_cmd = control(0, 1, gain, 0, 0, 0, 0, button_0, button_1)
            elif button_0 or button_1:
                control_cmd = control(0, 0, gain, 0, 0, 0, 0, button_0, button_1)
            else:
                control_cmd = control(0, 0, gain, 0, 0, 0, 0, 0, 0)
            rospy.loginfo(control_cmd)
            pub.publish(control_cmd)
            if not is_master_online():
                rospy.signal_shutdown("ROS master is offline")
            rate.sleep()


if __name__ == '__main__':
    try:
        send_control_cmd()
    except rospy.ROSInterruptException:
        pass
