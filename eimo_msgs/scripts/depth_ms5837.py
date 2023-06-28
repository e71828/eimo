import time

import ms5837
import rospy
from eimo_msgs.msg import depth


class I2cMs5837:
    def __init__(self):
        self.depth_publisher = rospy.Publisher('depth', depth, queue_size=10)
        rospy.init_node('publish_depth', anonymous=True)

        self.frequency = rospy.get_param('depth_frequency', default=1)
        self.i2c_port = rospy.get_param('~i2c_port', default='/dev/i2c-1')
        self.sensor = ms5837.MS5837_02BA(int(self.i2c_port[-1]))  # Default I2C bus is 1
        self.fluid_density = rospy.get_param('~density', default=897)
        self.sensor.setFluidDensity(self.fluid_density) # kg/m^3

        # We must initialize the sensor before reading it
        if not self.sensor.init():
            rospy.logerr("Depth sensor could not be initialized from i2c port {}".format(self.i2c_port))
            return
        else:
            rospy.loginfo("Opened i2c port {}".format(self.i2c_port))
            self.sensor.read(ms5837.OSR_8192)
            depth_value = 0
            for i in  range(6):
                depth_value += self.sensor.depth()
            self.init_depth = depth_value / 6
            rospy.loginfo('init m: {}'.format(self.init_depth))
            rospy.set_param('init_depth', int(self.init_depth*1000))
            self.pub_depth()
            pass

    def pub_depth(self):
        rate = rospy.Rate(self.frequency)  # 1hz
        while not rospy.is_shutdown():
            depth_value = 0
            # read depth 90 times and get the average value
            for _ in range(30):
                self.sensor.read(ms5837.OSR_512)
                depth_value += self.sensor.depth()
                time.sleep(0.01)
            depth_value /= 30
            depth_value_mm = int(depth_value * 1000 + 300)
            if depth_value_mm < 0:
                depth_value_mm = 0
            self.depth_publisher.publish(depth_value_mm)
            rospy.loginfo('mm: {}'.format(depth_value_mm))
            rate.sleep()


if __name__ == '__main__':
    try:
        I2cMs5837()
    except rospy.ROSInterruptException:
        pass
