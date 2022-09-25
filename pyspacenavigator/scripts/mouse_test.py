import rospy
from time import sleep
from pyspacenavigator import spacenavigator

if __name__ == '__main__':

    success = spacenavigator.open(callback=spacenavigator.print_state, button_callback=spacenavigator.toggle_led)
    # success = spacenavigator.open()

    if success:
        print("Devices found:\n\t%s" % "\n\t".join(spacenavigator.list_devices()))
        while 1:
            # state = spacenavigator.read()
            # print(state.x, state.y, state.z)
            sleep(0.5)
