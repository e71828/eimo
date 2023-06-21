import pyspacemouse
import rospy
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
        pub = rospy.Publisher('control', control, queue_size=10)
        rospy.init_node('publish_control_command', anonymous=True)
        frequency = rospy.get_param('~frequency', default=10)
        rate = rospy.Rate(frequency)  # 10hz
        threshold = 0.8
        while not rospy.is_shutdown():
            readings = []
            for _ in range(10):
                state = pyspacemouse.read()
                readings.append(state)

            # Compute average reading
            state = sum(readings) / 10
            control_cmd = control(state.y > threshold, state.y < -threshold, 255, state.z > threshold,
                                  state.z < -threshold,
                                  state.yaw < -0.8, state.yaw > 0.8, state.buttons[0], state.buttons[1])
            rospy.loginfo(control_cmd)
            pub.publish(control_cmd)
            rate.sleep()


if __name__ == '__main__':
    try:
        send_control_cmd()
    except rospy.ROSInterruptException:
        pass
