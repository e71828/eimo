import time

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
        pub = rospy.Publisher('control', control, queue_size=1)
        rospy.init_node('publish_control_command', anonymous=True)
        frequency = rospy.get_param('~frequency', default=8)
        rate = rospy.Rate(frequency)  # 10hz
        threshold = 0.8
        while not rospy.is_shutdown():
            readings = []
            for _ in range(7):
                state = pyspacemouse.read()
                readings.append(state)
                time.sleep(0.01)

            # 初始化求和变量
            sum_t = 0
            sum_x = 0
            sum_y = 0
            sum_z = 0
            sum_roll = 0
            sum_pitch = 0
            sum_yaw = 0
            sum_buttons_0 = 0
            sum_buttons_1 = 0

            # 对每个数据点进行求和
            for nav in readings:
                # sum_t += nav.t
                # sum_x += nav.x
                sum_y += nav.y
                sum_z += nav.z
                # sum_roll += nav.roll
                # sum_pitch += nav.pitch
                sum_yaw += nav.yaw
                sum_buttons_0 += nav.buttons[0]
                sum_buttons_1 += nav.buttons[1]

            # 计算平均值
            # avg_t = sum_t / len(readings)
            # avg_x = sum_x / len(readings)
            avg_y = sum_y / len(readings)
            avg_z = sum_z / len(readings)
            # avg_roll = sum_roll / len(readings)
            # avg_pitch = sum_pitch / len(readings)
            avg_yaw = sum_yaw / len(readings)
            avg_buttons_0 = sum_buttons_0 / len(readings)
            avg_buttons_1 = sum_buttons_1 / len(readings)

            control_cmd = control(avg_y > threshold, avg_y < -threshold, 64, avg_z > threshold,
                                  avg_z < -threshold,
                                  avg_yaw < -threshold, avg_yaw > threshold, avg_buttons_0 > 0.8, avg_buttons_1 > 0.8)
            rospy.loginfo(control_cmd)
            pub.publish(control_cmd)
            rate.sleep()


if __name__ == '__main__':
    try:
        send_control_cmd()
    except rospy.ROSInterruptException:
        pass
