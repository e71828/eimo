from pyspacenavigator import spacenavigator
import rospy
from eimo_msgs.msg import control


def send_cmd():
    print("Devices found:\n\t%s" % "\n\t".join(spacenavigator.list_devices()))
    dev = spacenavigator.open(callback=None, button_callback=spacenavigator.toggle_led)
    if dev:
        print(dev.describe_connection())
        state = spacenavigator.read()
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
        rospy.init_node('forward control message', anonymous=True)
        rate = rospy.Rate(10)  # 10hz
        threshold = 0.8
        while not rospy.is_shutdown():
            control_cmd = control(state.x > threshold, state.x < -threshold, 255, state.z > threshold,
                                  state.z < -threshold,
                                  state.yaw > 0.8, state.yaw < -0.8, state.buttons[0], state.buttons[1])
            rospy.loginfo(control_cmd)
            pub.publish(control_cmd)
            rate.sleep()


if __name__ == '__main__':
    try:
        send_cmd()
    except rospy.ROSInterruptException:
        pass
