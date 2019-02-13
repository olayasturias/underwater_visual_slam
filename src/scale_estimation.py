#!/usr/bin/env python
import rospy
import sys
import tf2_ros
from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure
import math
import numpy as np
from geometry_msgs.msg import Point, TransformStamped
import Queue

class ScaleEstimation(object):
    def __init__(self,barometer_topic = "/bar30/pressure", vo_topic= "/visual_odom",
                 fluid_density = 997, atm_pressure = 1024):
        barom_sub = rospy.Subscriber(barometer_topic,FluidPressure,
                                    self.barometer_callback, queue_size=2)
        odom_sub = rospy.Subscriber(vo_topic, Odometry,
                                    self.vodometry_callbak, queue_size = 2)
        self.P0 = atm_pressure
        self.rho = fluid_density
        self.barometer_queue = Queue.Queue(maxsize = 10)
        self.vodometry_array = [0.0,0.0]
        self.barometer_array = [0.0,0.0]
        self.barometer_covar = 0.0
        self.vodometry_covar = 0.0
        self.odom_pub = rospy.Publisher('/scaled_odom',Odometry, queue_size = 5)

    def queue_get_all(self,q):
        items = []
        maxItemsToRetreive = 10
        for numOfItemsRetrieved in range(0, maxItemsToRetreive):
            try:
                if numOfItemsRetrieved == maxItemsToRetreive:
                    break
                items.append(q.get_nowait())
            except:
                break
        return items

    def barometer_callback(self, data):
        self.barometer_queue.put(data.fluid_pressure)
        self.barometer_covar = data.variance
        if data.variance == 0:
            self.barometer_covar = 0.0787

    def vodometry_callbak(self,data):
        self.vodometry_array[1] = self.vodometry_array[0]
        self.vodometry_array[0] = data.pose.pose.position.z
        self.vodometry_covar = 0.319775 # in meters!!

        barometer = self.queue_get_all(self.barometer_queue)
        self.barometer_array[1] = self.barometer_array[0]
        # hydrostatic equation
        self.barometer_array[0] = (np.mean(barometer)-self.P0)/(self.rho*9.85)

        scaled_vo = self.scale_estimation(data.pose.pose.position)

        # Publish Odometry msg
        odom = Odometry()
        odom = data
        odom.header.stamp  = rospy.Time.now()
        odom.pose.pose.position = Point(scaled_vo[0],scaled_vo[1],scaled_vo[2])
        self.odom_pub.publish(odom)

        # Publish tf
        self.transf = TransformStamped()
        self.transf.header.stamp = rospy.Time.now()
        self.transf.header.frame_id = 'world'
        self.transf.child_frame_id = 'base_link'

        self.transf.transform.translation.x = scaled_vo[0]
        self.transf.transform.translation.y = scaled_vo[1]
        self.transf.transform.translation.z = scaled_vo[2]

        self.transf.transform.rotation = data.pose.pose.orientation
        # Create broadcaster and broadcast transform
        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(self.transf)



    def scale_estimation(self, vo_data):

        xi = self.vodometry_array[0]-self.vodometry_array[1]
        yi = self.barometer_array[0]-self.barometer_array[1]

        Sxx = self.barometer_covar**2*xi**2
        # print self.barometer_covar
        # print self.barometer_covar**2
        # print xi**2
        # print 'sxx', Sxx
        Syy = self.vodometry_covar**2*yi**2
        Sxy = self.barometer_covar*self.vodometry_covar*xi*yi

        sqrterm = math.sqrt((Sxx-Syy)**2+4*Sxy**2)

        numerator = Sxx-Syy+np.sign(Sxy)*sqrterm
        denominator = 2*Sxy*self.barometer_covar/self.vodometry_covar

        scale = numerator/denominator

        numerator = scale*self.barometer_covar**2*xi + self.vodometry_covar**2*yi
        denominator = scale**2*self.barometer_covar**2+self.vodometry_covar

        estimate = numerator/denominator

        vo_scale = estimate / vo_data.z

        scaled_vo = np.array([vo_data.x, vo_data.y, vo_data.z])*vo_scale

        return scaled_vo





def main(args):
    # main
    rospy.init_node('scale_est', anonymous=True,log_level=rospy.DEBUG)
    rate = rospy.Rate(1) # 1 Hz

    # Read parameters from console (o launch file)
    # old_frame    = rospy.get_param("~old_frame")
    # new_frame    = rospy.get_param("~new_frame")
    odom_topic  = rospy.get_param("~odom_topic")
    barometer_topic  = rospy.get_param("~barom_topic")
    atm_pressure  = float(rospy.get_param("~atm_pressure"))
    fluid_density  = float(rospy.get_param("~fluid_density"))

    sc = ScaleEstimation(barometer_topic=barometer_topic,vo_topic= odom_topic,
                         fluid_density=fluid_density, atm_pressure = atm_pressure)



    rospy.spin()


    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
