#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure

class StateEstimation(object):
    def __init__(self,barometer_topic,vo_topic):
        barom_sub = rospy.Subscriber(barometer_topic,FluidPressure,
                                    self.barometer_callback, queue_size=2)
        odom_sub = rospy.Subscriber(vo_topic, Odometry,
                                    self.vodometry_callbak, queue_size = 2)
        self.barometer_array = []
        self.vodometry_array = [0,0]


def main(args):
    # main
    rospy.init_node('scale_est', anonymous=True,log_level=rospy.DEBUG)
    rate = rospy.Rate(1) # 1 Hz

    # Read parameters from console (o launch file)
    # old_frame    = rospy.get_param("~old_frame")
    # new_frame    = rospy.get_param("~new_frame")
    odom_topic  = rospy.get_param("~odom_topic")
    barometer_topic  = rospy.get_param("~barom_topic")




    while not rospy.is_shutdown():
        rospy.sleep(rate)
        pass


    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
