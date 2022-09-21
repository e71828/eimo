from pyspacenavigator import spacenavigator
import rospy
from eimo_msgs.msg import control
from time import sleep


class MouseControl:

    def __init__(self):
        self.mouse_publisher = rospy.Publisher('control', control, queue_size=10)
        rospy.init_node('forward_control_command_after_trigger', anonymous=True)

        print("Devices found:\n\t%s" % "\n\t".join(spacenavigator.list_devices()))
        self.dev = spacenavigator.open(callback=self.send_after_trigger)
        if self.dev:
            self.dev.set_led(0)
            while not rospy.is_shutdown():
                sleep(1)
                self.dev.set_led(1)
                sleep(1)
                self.dev.set_led(0)
        else:
            pass

    def send_after_trigger(self, state):
        threshold = 0.8
        control_cmd = control(state.y > threshold, state.y < -threshold, 255, state.z > threshold,
                              state.z < -threshold,
                              state.yaw < -0.8, state.yaw > 0.8, state.buttons[0], state.buttons[1])
        rospy.loginfo(control_cmd)
        self.mouse_publisher.publish(control_cmd)

    @property
    def state(self):
        """Return the current value of read()
        Returns: state: {t,x,y,z,pitch,yaw,roll,button} namedtuple
                None if the device is not open.
        """
        return self.dev.read()


if __name__ == '__main__':
    try:
        MouseControl()
    except rospy.ROSInterruptException:
        pass
