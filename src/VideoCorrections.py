#!/usr/bin/env python
import rospy
import sys,os
import cv2
#assert cv2.__version__[0] == '3', 'The fisheye module requires opencv version >= 3.0.0'
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import threading
import queue
import gc
import time

class VideoCorrect():
    def __init__(self):
        self.H = []
        self.nrows = 0
        self.ncols = 0

        self.img_pub = rospy.Publisher('/BlueRov2/image_corrected',
                                       Image,
                                       queue_size=1)
        self.cam_info_sub = rospy.Subscriber('/BlueRov2/camera_info',
                                            CameraInfo,
                                            self.info_callback,queue_size=1)


    def filter(self,img):
        undistorted = self.correct(img)

        #small = cv2.resize(undistorted,(0,0), fx=0.75, fy=0.75)
        small = undistorted
        del undistorted


        img32  = np.float32(small)
        img_norm = img32/255
        del small

        yuv  = cv2.cvtColor(img_norm,cv2.COLOR_BGR2YUV)
        del img_norm

        y = yuv[:, :, 0]
        u = yuv[:, :, 1]
        v = yuv[:, :, 2]
        rows, cols = y.shape
        nrows = cv2.getOptimalDFTSize(rows)
        ncols = cv2.getOptimalDFTSize(cols)

        # High pass filter for the homomorphic FILTER
        # This calculation is costly, so that's why we take the operation out
        # of the homomorphic function

        if nrows != self.nrows or ncols != self.ncols:
            self.nrows = nrows
            self.ncols = ncols
            rh, rl, cutoff = 2.5,0.5,32

            DX = cols/cutoff

            F = []
            n = 100000.0
            F = [(-((i-self.nrows/2)**2+(j-self.ncols/2)**2))
                 for i in range (self.nrows) for j in range (self.ncols)]
            H = np.asarray(F,dtype=np.float64)/(2.0*DX**2)
            H = H/n
            H = (1-(1+H)**n)
            H = H*(rh-rl)
            H = H +rl
            self.H = np.reshape(H,(self.nrows,self.ncols))


        corrected = self.fast_homomorphic_filter(y)
        masked = self.circle_mask(corrected)
        bridge = CvBridge()
        image_message = bridge.cv2_to_imgmsg(masked, encoding="mono8")
        self.img_pub.publish(image_message)
        del image_message

        gc.collect()
        return masked

    def info_callback(self, data):
        self.K = np.reshape(data.K,(3,3))
        self.R = np.reshape(data.R,(3,3))
        self.D = data.D
        self.width = data.width
        self.height = data.height
        self.newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.K,# camera matrix
                                                          self.D,# distorsion coef
                                                          (data.width,data.height),
                                                          0,# alpha
                                                          (data.width,data.height))
        self.cam_info_sub.unregister()


    def correct(self, img):
        try:
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(self.K,
                                                                self.D[-1],
                                                                self.R,
                                                                self.newcameramtx,
                                                                (self.width,self.height),
                                                                cv2.CV_16SC2)
            undistorted_img = cv2.remap(img, map1, map2,
                                        interpolation=cv2.INTER_LINEAR,
                                        borderMode=cv2.BORDER_CONSTANT)

            return undistorted_img
        except Exception as e:
            return img


    def circle_mask(self,img):
        mask = np.zeros(img.shape[:2], np.uint8)
        h, w = img.shape[:2]
        # circle center
        hc = int(h*0.33)
        wc = int(w*0.45)
        radius = int(w*0.44)
        cv2.circle(mask,(wc,hc),radius,255,-1)
        masked_img = cv2.bitwise_and(src1 = img,src2 = img, mask = mask)


        return masked_img


    def fast_homomorphic_filter(self, img):
        '''
        Code adapted from here :
        https://sites.google.com/site/bazeilst/tutorials#TUTO9
        As adapted from here :
        https://dsp.stackexchange.com/questions/42476/homomorphic-filter-python-overflow
        '''
        rospy.logdebug('Entering homomorphic filter')
        rows,cols = img.shape
        nrows = self.nrows
        ncols = self.ncols

        y  = img

        ## LOGARITHM
        # Logarithm of image so it can be represented as a product of reflectance and illumination
        # Computed with approximation by limits

        n = 100000.0
        y_log = n* (((y) ** (1/n)) - 1)
        del y

        ## FOURIER TRANSFORM
        # Represent image in frequency domain


        # Generate copy of image with optimized size
        nimg = np.zeros((nrows,ncols))
        nimg[:rows,:cols] = y_log

        # Do the DFT itself
        y_fft= cv2.dft(np.float32(nimg),flags=cv2.DFT_COMPLEX_OUTPUT)
        y_fft_shift = np.fft.fftshift(y_fft)
        del y_fft

        ## HIGH PASS FILTER

        dft_complex = y_fft_shift[:,:,1] + y_fft_shift[:,:,0]*1j
        result_filter = self.H * dft_complex
        del dft_complex

        ## INVERSE FOURIER TRANSFORM

        idft_flags = cv2.DFT_COMPLEX_OUTPUT| cv2.DFT_SCALE | cv2.DFT_REAL_OUTPUT

        result_split = np.ndarray(y_fft_shift.shape)
        del y_fft_shift
        result_split[:,:,0] = result_filter.imag
        result_split[:,:,1] = result_filter.real

        result_interm = cv2.idft(np.fft.ifftshift(result_split),flags = idft_flags)
        del result_split

        ## EXPONENTIAL OF IMAGE
        ## To revert the logarithm
        ## Sometimes high values appear, so we use clip to limit them

        result = np.exp(np.clip(result_interm,np.amin(result_interm),0))
        del result_interm
        y = result[0:rows,0:cols]
        del result


        y = np.uint8(y*255)

        median = cv2.medianBlur(y,5)
        del y

        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        cl1 = clahe.apply(median)
        del median

        return cl1
