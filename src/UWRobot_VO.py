#!/usr/bin/env python
import rospy
import sys
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from VisualOdometry import VisualOdometry
import gc
import pandas as pd
import time
from sensor_msgs.msg import Image

"""
.. codeauthor:: Olaya
: file UWrobot_VO.py.py
"""


class UWVisualOdometry(object):
    def __init__(self, params, imgtopic):
        # Old and new imgs
        self.old_frame = np.array([])
        self.new_frame = np.array([])

        log = pd.DataFrame([['','','','']],
                           columns = ['feature','nmatches','ngoodmatches','ex_time'])
        log.to_csv('~/catkin_ws/volog.csv')

        # Create Visual Odometry instance
        self.vo = VisualOdometry(params)

        self.image_sub = rospy.Subscriber(imgtopic, Image, self.img_callback, queue_size = 1)
        rospy.loginfo("Subscribed to %s topic", imgtopic)

    def img_callback(self,data):
        """ Receives the Image topic message and converts it from sensor_msgs
        to cv image using cv_bridge.
        """
        rospy.logdebug('Received image topic...')
        bridge = CvBridge()
        try:
            # Read and convert data
            current_frame = bridge.imgmsg_to_cv2(data, "mono8")

            self.new_frame = current_frame
            # If first iteration, copy image twice
            if not self.old_frame.size:
                self.old_frame = self.new_frame.copy()
            start = time.time()
            # Do VO stuff
            self.vo.init_reconstruction(optimize = False,
                                        image1 = self.old_frame,
                                        image2 = self.new_frame)

            # Log time  taken
            end = time.time()
            ttaken = end-start

            saved = pd.read_csv('~/catkin_ws/volog.csv',index_col=0,header=0)

            new = pd.DataFrame([['orb',len(self.vo.matcher.matches1),
                                 len(self.vo.matcher.good_matches),ttaken]],
                               columns = ['feature','nmatches','ngoodmatches','ex_time'])

            log = pd.concat([saved,new])
            print log
            log.to_csv('~/catkin_ws/volog.csv')
            # Print things
            # imgmatch = self.new_frame.copy()
            # self.vo.matcher.draw_matches(img = imgmatch,
            #                              matches = self.vo.matcher.good_matches)
            # rospy.logdebug("# good matches: {}".format(len(self.vo.matcher.good_matches)))
            # cv2.imshow('matches', imgmatch)
            # cv2.waitKey(3)


            # Now the new frame becomes the older one
            self.old_frame = self.new_frame.copy()
            gc.collect()
        except CvBridgeError as e:
            print(e)



def main(args):
    # main
    rospy.init_node('rov_vo', anonymous=True,log_level=rospy.DEBUG)
    rate = rospy.Rate(1) # 1 Hz

    # Read parameters from console (o launch file)
    # old_frame    = rospy.get_param("~old_frame")
    # new_frame    = rospy.get_param("~new_frame")
    input_topic  = rospy.get_param("~img_topic")

    parameters = {"detector" : 'sift',"matcher" : 'bf'}

    # init only defines matcher object
    orb_vo = UWVisualOdometry(parameters,input_topic)
    #orb_bfmatcher.match(feature1,feature2)



    while not rospy.is_shutdown():
        pass


    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
