#!/usr/bin/env python
import time, sys, os
from ros import rosbag
import roslib, rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import CompressedImage
import numpy as np
#from cv_bridge import CvBridge
import cv2

TOPIC = '/BlueRov2/image/compressed'

def CreateVideoBag(videopath, bagname):
    '''Creates a bag file with a video file'''
    bag = rosbag.Bag(bagname, 'w')
    cap = cv2.VideoCapture(videopath)
    #cb = CvBridge()
    print cap.get(3)
    print cap.get(4)
    prop_fps = cap.get(cv2.CAP_PROP_FPS)
    if prop_fps != prop_fps or prop_fps <= 1e-2:
        print "Warning: can't get FPS. Assuming 24."
        prop_fps = 24
    ret = True
    frame_id = 0
    while(ret):
        ret, frame = cap.read()
        frame = frame[:,130:890]
        cv2.imshow('Frame',frame[:,130:890])
        cv2.waitKey(3)
        if not ret:
            break
        stamp = rospy.rostime.Time.from_sec(float(frame_id) / prop_fps)
        frame_id += 1
        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        #image = cb.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.frame_id = "BlueRov2Camera"
        bag.write(TOPIC, msg, stamp)
    cap.release()
    bag.close()

if __name__ == "__main__":
    if len( sys.argv ) == 3:
        CreateVideoBag(*sys.argv[1:])
    else:
        print( "Usage: video2bag videofilename bagfilename")
