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
import datetime
from sensor_msgs.msg import Image
from VideoCorrections import VideoCorrect

"""
.. codeauthor:: Olaya
: file UWrobot_VO.py.py
"""


class UWVisualOdometry(object):
    def __init__(self, params, imgtopic):
        # Old and new imgs
        self.old_frame = np.array([])
        self.new_frame = np.array([])

        #form str for csv file name
        self.strfile = '~/catkin_ws/' + params['detector'] + params['matcher'] +str(datetime.datetime.now()) + '.csv'

        log = pd.DataFrame(columns = ['nfeatures','nmatches','ngoodmatches','reproj_error','ex_time'])
        log.to_csv(self.strfile)
        rospy.loginfo('saving log file in %s',self.strfile)

        # Create filter image instance for video VideoCorrections
        self.videocorrect = VideoCorrect()

        # Create Visual Odometry instance
        self.vo = VisualOdometry(params)

        self.image_sub = rospy.Subscriber(imgtopic, Image, self.img_callback, queue_size = 1)
        rospy.loginfo("Subscribed to %s topic", imgtopic)

        self.img_pub = rospy.Publisher('/BlueRov2/image_matches',
                                       Image,
                                       queue_size=1)


    def img_callback(self,data):
        """ Receives the Image topic message and converts it from sensor_msgs
        to cv image using cv_bridge.
        """
        rospy.logdebug('Received image topic...')
        start = time.time()
        bridge = CvBridge()
        # Read and convert data
        current_frame = bridge.imgmsg_to_cv2(data, "bgr8")

        corrected = self.videocorrect.filter(current_frame)
        end = time.time()
        rospy.logdebug('Time taking for homomorphic %s',end-start)

        self.new_frame = corrected
        # If first iteration, copy image twice
        if not self.old_frame.size:
            self.old_frame = self.new_frame.copy()

        # Prepare csv for log file
        saved = pd.read_csv(self.strfile,index_col=0,header=0)
        self.vo.cam.K = self.videocorrect.K
        try:
            # Do VO stuff.. If you can!!
            # try:
            self.vo.init_reconstruction(optimize = False,
                                            image1 = self.old_frame,
                                            image2 = self.new_frame)
            # Log time  taken
            end = time.time()
            ttaken = end-start

            rospy.loginfo('VO done in %s s', ttaken)

            new = pd.DataFrame([[len(self.vo.matcher.kp1),len(self.vo.matcher.matches1),
                                     len(self.vo.matcher.good_matches),self.vo.reprojection_error_mean,ttaken]],
                                   columns = ['nfeatures','nmatches','ngoodmatches','reproj_error','ex_time'])
        except:
            rospy.logwarn('Not enough matches in this pair of frames')
            end = time.time()
            ttaken = end-start
            if not self.vo.matcher.kp1:
                nkp1 = 0
            else:
                nkp1 = len(self.vo.matcher.kp1)
            if not self.vo.matcher.matches1:
                nmatches = 0
            else:
                nmatches = len(self.vo.matcher.matches1)
            if not self.vo.matcher.good_matches:
                ngood = 0
            else:
                ngood = len(self.vo.matcher.good_matches)
            if not self.vo.reprojection_error_mean:
                repr_err = 0
            else:
                repr_err = self.vo.reprojection_error_mean
            new = pd.DataFrame([[ nkp1, nmatches,
                                  ngood, repr_err ,ttaken]],
                                   columns = ['nfeatures','nmatches',
                                              'ngoodmatches','reproj_error',
                                              'ex_time'])

        log = pd.concat([saved,new])
        log.to_csv(self.strfile)

        # Print things
        imgmatch = self.new_frame.copy()
        self.vo.matcher.draw_matches(img = imgmatch,
                                    matches = self.vo.matcher.good_matches)
        # rospy.logdebug("# good matches: {}".format(len(self.vo.matcher.good_matches)))
        # cv2.imshow('matches', imgmatch)
        # cv2.waitKey(3)
        # Now the new frame becomes the older one
        self.old_frame = self.new_frame.copy()

        image_message = bridge.cv2_to_imgmsg(imgmatch, encoding="mono8")
        del imgmatch
        self.img_pub.publish(image_message)
        del image_message

        rospy.loginfo('EXITING IMG CALLBACK')


        #gc.collect()




def main(args):
    # main
    rospy.init_node('rov_vo', anonymous=True,log_level=rospy.DEBUG)
    rate = rospy.Rate(500) # 1 Hz

    # Read parameters from console (o launch file)
    # old_frame    = rospy.get_param("~old_frame")
    # new_frame    = rospy.get_param("~new_frame")
    input_topic  = rospy.get_param("~img_topic")

    parameters = {"detector" : 'orb',"matcher" : 'bf'}

    # init only defines matcher object
    orb_vo = UWVisualOdometry(parameters,input_topic)
    #orb_bfmatcher.match(feature1,feature2)



    rospy.spin()
    rospy.loginfo('Closing Visual Odometry Node.........')
    rospy.loginfo('bye bye')


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
