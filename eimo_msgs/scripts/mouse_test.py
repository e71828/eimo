import rospy

from pyspacenavigator import spacenavigator
if __name__=='__main__':
    rospy.init_node('test_node')
    succ = pyspacenavigator.open()
    print(succ)
# if success:
#     print("Devices found:\n\t%s" % "\n\t".join(spacenavigator.list_devices()))
#     while 1:
#         state = spacenavigator.read()
#         print(state.x, state.y, state.z)
#         time.sleep(0.5)
