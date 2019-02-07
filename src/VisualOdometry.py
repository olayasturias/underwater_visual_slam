#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from scipy import linalg
from scipy import optimize
from math import cos, sin, exp
import os
from Matcher import Matcher
from Camera import Camera
from Map import Map
from MapPoint import MapPoint
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion
from pyquaternion import Quaternion as pyquat

"""
.. codeauthor:: Olaya
: file VisualOdometry.py
"""



class VisualOdometry(object):
    """ The **VisualOdometry** class contains all the required methods to
    recover the motion of the camera and the structure of the scene.

    This class has as an attribute a Dataset class instance and a Matcher
    class instance, in order to make its use easier. The algorithms implemented
    here (most of all) follow those explained in the excellent book *Multiple
    View Geometry*, written by R.Hartley and A.Zisserman (HZ_)

    **Attributes**:

        .. data:: F

           The estimated Fundamental_  matrix. Numpy ndarray (3x3).

        .. data:: E

           The estimated Essential_  matrix. Numpy ndarray (3x3).

        .. data:: H

           The estimated planar Homography_ matrix. Numpy ndarray(3x3)

        .. data:: right_e

           Right Epipole

        .. data:: left_e

           Left epipole

        .. data:: cam

           The Camera instance (**for the current frame**)

           .. seealso::

               Class :py:class:`Camera.Camera`

        .. data:: structure

           3D triangulated points (Numpy ndarray nx3)

        .. data:: mask

           Numpy array. Every element of this array which is zero is suposed
           to be an outlier. These attribute is used by the
           *FindFundamentalRansac* and *FindEssentialRansac* methods, and
           can be used to reject the KeyPoints outliers that remains after
           the filtering process.

        .. data:: index

           This parameter count the number of iterations already done by the
           system. **Whenever we iterate over the dataset (i.e, read a new image
           and recover the structure, etc) we have to increase by two this
           parameter, so we can index correctly the camera matrices**. For example,
           at the beginning it will be 0, so the first camera will be stored
           in the first position of the list of cameras, and the second one in
           the second position (0 + 1). Next, we read a new image (so the new
           one will be in the *image_2* attribute of the Dataset instance, and
           the last one will be stored in the *image_1* attribute), and increase
           the index by two, so now the *previous frame* camera matrix will be
           stored in the third position (2) and the *current frame* in the
           fourth position (4), and so on.

        .. data:: matcher

           Instance of the matcher class

           .. seealso::

               :py:mod:`Matcher`

        .. data:: scene

           Instance of the Map class. The scene as seen by the camera.

           .. seealso::

               :py:class:`Map.Map`

        .. data:: delta_t

            Incremental traslation in camera coordinates

        .. data:: delta_R

            Incremental rotation

    **Constructor**:

        The constructor has one optional parameter:

            1. The Matcher parameters. If no parameters are provided, the
               system will use ORB as detector and a Brute-Force based matcher.

               .. seealso::

                   Class :py:mod:`Matcher`

    .. _HZ: http://www.robots.ox.ac.uk/~vgg/hzbook/
    .. _Fundamental: https://en.wikipedia.org/wiki/Fundamental_matrix_(computer_vision)
    .. _Essential: https://en.wikipedia.org/wiki/Essential_matrix
    .. _Homography: https://en.wikipedia.org/wiki/Homography_(computer_vision)
    .. _RANSAC: http://www.cs.columbia.edu/~belhumeur/courses/compPhoto/ransac.pdf
    .. _findFundamentalMat: http://docs.opencv.org/3.0-beta/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#findfundamentalmat
    .. _Nister: http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.86.8769&rep=rep1&type=pdf


    """
    def __init__(self, params=None):
        """ Constructor

        """
        if params is None:
            params = dict(detector='orb',
                          matcher='bf')
        self.matcher = Matcher(params)
        self.F = None
        self.mask = None
        self.H = None
        self.right_e = None
        self.left_e = None
        self.cam = Camera()
        self.E = None  # Essential matrix
        self.index = 0
        self.scene = Map()
        self.delta_t = 0.0
        self.delta_R = np.identity(3)
        self.odom_pub = rospy.Publisher('/visual_odom',Odometry, queue_size = 5)

    def init_reconstruction(self, optimize=True, image1 = None, image2 = None):
        """ Performs the first steps of the reconstruction.

        The first steps are:

            1. Read the two first images and match them.
            2. Get an initial estimate of the Fundamental matrix and reject
               outliers.
            3. Reestimate the Fundamental matrix without outliers.
            4. Triangulate the image points up to a projective transformation.
            5. Optimize the Fundamental matrix by minimizing the reprojection
               error.
            6. Triangulate the image points up to a scale factor.
            7. Filter out the 3D points behind the camera and too far from it.
            8. Init the map.

        :param optimize: If True performs nonlinear optimization of :math:`F`
        :type optimize: Boolean

        """
        # 1
        self.matcher.match(image1, image2)
        # 2
        self.kp1 = np.asarray([item.pt for item in self.matcher.good_kp1])
        self.kp2 = np.asarray([item.pt for item in self.matcher.good_kp2])
        self.FindFundamentalRansac(self.kp1, self.kp2,'RANSAC')
        self.reject_outliers()
        self.kp1 = np.asarray([item.pt for item in self.matcher.good_kp1])
        self.kp2 = np.asarray([item.pt for item in self.matcher.good_kp2])
        # 3
        self.FindFundamentalRansac(self.kp1,self.kp2,'RANSAC')


        if not optimize:
            # 4
            self.structure = self.triangulate(self.kp1, self.kp2, euclidean = True)
            reproj_mask = self.mask_reprojection(self.cam.K, self.cam.R, self.cam.t,
                                                 self.structure,self.kp2)
            if any(reproj_mask):
                self.kp1 = self.kp1[reproj_mask]
                self.kp2 = self.kp2[reproj_mask]
                self.matcher.good_desc1 = np.asarray(self.matcher.good_desc1)[reproj_mask]
                self.matcher.good_desc2 = np.asarray(self.matcher.good_desc2)[reproj_mask]
                self.FindFundamentalRansac(self.kp1,self.kp2,'RANSAC')
            else:
                optimize = True

        if optimize:
            # 4
            self.structure = self.triangulate(self.kp1, self.kp2)
            # 5
            if (12+len(self.structure))<(len(self.kp1+len(self.kp2))):
                sol, F = self.optimize_F(self.kp1, self.kp2, self.F, self.structure)
                self.F = F

        # 6
        self.structure = self.triangulate(self.kp1, self.kp2, euclidean=True)
        # 7
        self.structure, mask = self.filter_z(self.structure)

        self.kp1 = self.kp1[mask]
        self.kp2 = self.kp2[mask]
        desc1 = np.asarray(self.matcher.good_desc1)[mask]
        desc2 = np.asarray(self.matcher.good_desc2)[mask]


        # 8
        cam1 = Camera()
        cam1.set_index(self.index)
        cam1.set_P(np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]))
        # cam1.is_keyframe()
        cam1.set_points(self.kp1)
        cam1.set_descriptors(desc1)
        self.cam.set_index(self.index + 1)
        self.cam.set_points(self.kp2)
        self.cam.set_descriptors(desc2)
        # 9
        for i in range(len(self.structure)):
            descriptors = np.vstack((desc1[i],
                                     desc2[i]))
            points = np.vstack((self.kp1[i],
                                self.kp2[i]))
            self.scene.add_mappoint(MapPoint(self.structure[i, :],
                                             [cam1, self.cam],
                                             points,
                                             descriptors))
        self.scene.add_camera(cam1)
        self.scene.add_camera(self.cam)
        self.cam.is_keyframe()
        self.index += 1

        # Increment Visual Odometry measurements
        self.delta_R = np.matmul(self.delta_R, self.cam.R)
        # Increments done on deltaR to avoid singularities and discontinuities.
        # This has the effect that different values could represent the same
        # rotation, for example quaternion q and -q represent the same rotation.
        # It is therefore possible that, when converting a rotation sequence,
        # the output may jump between these equivalent forms.
        # This could cause problems where subsequent operations such as
        # differentiation are done on this data.
        quaternion = pyquat(matrix = self.delta_R).elements
        self.delta_t += self.cam.t

        # Publish
        odom = Odometry()
        odom.header.stamp  = rospy.Time.now()
        odom.header.frame_id = 'vodom'
        odom.child_frame_id = 'base_link'
        # pyquaternion uses different order than ROS
        odom.pose.pose = Pose(Point(self.delta_t[0],self.delta_t[1],self.delta_t[2]),
                              Quaternion(quaternion[3],quaternion[0],
                                        quaternion[1], quaternion[2]))

        self.odom_pub.publish(odom)

    def track_local_map(self):
        """ Tracks the local map.

        This method use the *index* attribute to retrieve the local map points
        and tries to track them in successive frames. The algorithm is as
        follows:

            1. Using the Lucas-Kanade algorithm track the local map points in
               the new frame.
            2. If the tracked map points are less than 50, then exit and
               perform again the first step of the main algorithm.
               (see :py:func:`VisualOdometry.VisualOdometry.init_reconstruction`)
            3. If we have been tracking the local map for more than 10 frames
               then exit this method and perform again the first step of the
               main algorithm.
            4. With the tracked map points estimate the Fundamental matrix, and
               from F the motion of the camera.
            5. Project non-tracked map points and look for a correspondence
               in the new frame, within a image patch centered in
               its coordinates.
            6. Using the map points tracked in 1 and 5 reestimate the
               Fundamental matrix.
            7. Perform bundle adjustment (motion only) using the tracked map
               points.

        """
        self.kitti.read_image()
        previous_image = self.kitti.image_1.copy()
        points = self.cam.points
        for i in range(4):
            # 1
            mask, lk_prev_points, lk_next_points = self.matcher.lktracker(previous_image,
                                                                          self.kitti.image_2,
                                                                          points)
            print ("Tracked points: {}".format(len(lk_next_points)))
            # 2
            F = self.FindFundamentalRansac(lk_next_points, points[mask])
            E = self.E_from_F(F)
            pts1 = (np.reshape(points[mask], (len(points[mask]), 2))).T
            pts2 = (np.reshape(lk_next_points, (len(lk_next_points), 2))).T
            R, t = self.get_pose(pts1.T, pts2.T,
                                 self.cam.K, E)
            cam = Camera()
            cam.set_R(R)
            cam.set_t(t)
            cam.Rt2P(inplace=True)
            self.scene.add_camera(cam)
            self.kitti.read_image()
        return mask, lk_prev_points, lk_next_points

    def FindFundamentalRansac(self, kpts1, kpts2, method=cv2.FM_RANSAC, tol=1):
        """ Computes the Fundamental matrix from two set of KeyPoints, using
        a RANSAC_ scheme.

        This method calls the OpenCV findFundamentalMat_ function. Note that
        in order to compute the movement from the previous frame to the current
        one we have to invert the parameters *kpts1* (points in the previous
        frame) and *kpts2* (points in the current frame).


        :param kpts1: KeyPoints from the previous frame
        :param kpts2: KeyPoints from the current frame

        :param method: Method used by the OpenCV function to compute the
                       Fundamental matrix. It can take the following values:

                           * SEVEN_POINT, 7-Point algorithm
                           * EIGHT_POINT, 8-Point algorithm
                           * RANSAC, 8-Point or 7-Point (depending on the number
                             of points provided) algorithm within a RANSAC
                             scheme
                           * LMEDS, Least Median Squares algorithm

                     For more information about these algorithms see HZ_.

        :param tol: Pixel tolerance used by the RANSAC algorithm. By default 1.
        :type kpts1: Numpy ndarray nx2 (n is the number of KeyPoints)
        :type kpts2: Numpy ndarray nx2 (n is the number of KeyPoints)
        :type method: String
        :type tol: Integer

        :returns: The estimated Fundamental matrix (3x3) and an output array of
                  the same length as the input KeyPoints. Every element of this
                  array which is set to zero means that it is an **outlier**.
        :rtype: Numpy ndarray

        """
        algorithms = dict(SEVEN_POINT=cv2.FM_7POINT,
                          EIGHT_POINT=cv2.FM_8POINT,
                          RANSAC=cv2.FM_RANSAC,
                          LMEDS=cv2.FM_LMEDS)
        kpts1 = np.float32(kpts1)
        kpts2 = np.float32(kpts2)
        if method == 'RANSAC':
            try:
                self.F, self.mask = cv2.findFundamentalMat(kpts2,
                                                           kpts1,
                                                           algorithms[method],
                                                           tol)
                return self.F
            except Exception, e:
                print "Exception"
                print e
        else:
            try:
                self.F, self.mask = cv2.findFundamentalMat(kpts2,
                                                           kpts1,
                                                           algorithms[method])
                return self.F
            except Exception, e:
                print e

    def reject_outliers(self):
        """ Rejects the KeyPoints outliers.

        This method removes those KeyPoints marked as outliers by the mask
        returned by the *FindEssentialRansac* and *FindFundamentalRansac*
        methods.

        """
        if self.mask is None:
            pass
        else:
            msk_lst = self.mask.tolist()
            self.matcher.good_kp1 = [d for d, s in zip(self.matcher.good_kp1,
                                                       msk_lst) if s[0] == 1]
            self.matcher.good_desc1 = [d for d, s in zip(self.matcher.good_desc1,
                                                         msk_lst) if s[0]==1]
            self.matcher.good_kp2 = [d for d, s in zip(self.matcher.good_kp2,
                                                       msk_lst) if s[0] == 1]
            self.matcher.good_desc2 = [d for d, s in zip(self.matcher.good_desc2,
                                                         msk_lst) if s[0]==1]
            self.matcher.good_matches = [d for d, s in zip(self.matcher.good_matches,
                                                           msk_lst) if s[0] == 1]

    def draw_epilines(self, img1, img2, lines, pts1, pts2):
        """ Draw epilines in img1 for the points in img2 and viceversa

        :param img1: First image
        :param img2: Second image
        :param lines: Corresponding epilines
        :param pts1: KeyPoints in the first image (Integer values)
        :param pts2: KeyPoints in the second image (Integer values)
        :type img1: Numpy ndarray
        :type img2: Numpy ndarray
        :type lines: Numpy ndarray
        :type pts1: Numpy ndarray
        :type pts2: Numpy ndarray

        :returns: Two new images
        :rtype: Numpy ndarray


        """
        r, c = img1.shape
        # The next two lines don't work because the Kitti images
        # don't have color, so we can't convert them to BGR
        # img1 = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
        # img2 = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
        for r, pt1, pt2 in zip(lines, pts1, pts2):
            color = tuple(np.random.randint(0, 255, 3).tolist())
            x0, y0 = map(int, [0, -r[2]/r[1]])
            x1, y1 = map(int, [c, -(r[2]+r[0]*c)/r[1]])
            img1 = cv2.line(img1, (x0, y0), (x1, y1), color, 1)
            img1 = cv2.circle(img1, tuple(pt1.astype(int)), 5, color, -1)
            img2 = cv2.circle(img2, tuple(pt2.astype(int)), 5, color, -1)
        return img1, img2

    def find_epilines(self, pts):
        """ Find epilines corresponding to points in an image (where we have
        extracted *pts*) ready to plot in the other image.

        :param pts: KeyPoints of the image for which we are drawing its
                    epilines in the other image.
        :type pts: Numpy ndarray
        :returns: The epilines
        :rtype: Numpy ndarray
        """
        lines = cv2.computeCorrespondEpilines(pts.reshape(-1, 1, 2),
                                              2,
                                              self.F)
        lines = lines.reshape(-1, 3)
        return lines

    def FindEssentialRansac(self, kpts1, kpts2):
        # Compute Essential matrix from a set of corresponding points
        # @param kpts1: list of keypoints of the previous frame
        # @param kpts2: list of keypoints of the current frame

        kpts1 = np.float32(kpts1)
        kpts2 = np.float32(kpts2)

        # findEssentialMat takes as arguments, apart from the keypoints of both
        # images, the focal length and the principal point. Looking at the
        # source code of this function
        # (https://github.com/Itseez/opencv/blob/master/modules/calib3d/src/five-point.cpp)
        # I realized that these parameters are feeded to the function because it
        # internally create the camera matrix, so they must be in pixel
        # coordinates. Hence, we take them from the already known camera matrix:

        focal = 3.37
        pp = (2.85738, 0.8681)

        # pp = (self.K[0][2], self.K[1][2])

        self.E, self.maskE = cv2.findEssentialMat(kpts2, kpts1, focal, pp,
                                                  cv2.RANSAC, 0.999, 1.0,
                                                  self.maskE)

    def FindHomographyRansac(self, kpts1, kpts2):
        # Find the homography between two images given corresponding points
        kpts1 = np.float32(kpts1)
        kpts2 = np.float32(kpts2)

        self.H, self.maskH = cv2.findHomography(kpts1, kpts2, cv2.RANSAC, 1.0)

    def get_epipole(self, F=None):
        """ Computes the **right** epipole (:math:`\\mathbf{e}`). As it is the
        right null-vector of F, it satisfies

        .. math::

            F\\mathbf{e} = \\mathbf{0}

        If we want to compute the **left** epipole (:math:`\\mathbf{e'}`), then
        pass :math:`F^{t}`, because it is the left null-vector of F:

        .. math::

            F^{t}\\mathbf{e'} = \\mathbf{0}


        :param F: Fundamental matrix associated with the required epipoles.
                  If None, (by default) then it uses the class *F* attribute.
        :type F: Numpy 3x3 ndarray
        :returns: The right epipole associated with F
        :rtype: Numpy 1x3 ndarray

        """
        U, S, V = linalg.svd(F)
        e = V[-1]
        e = e / e[2]
        return e

    def skew(self, a):
        """ Return the matrix :math:`A` such that :math:`\\mathbf{a}` is its
        null-vector (right or left), i.e, its a 3x3 *skew-symmetric matrix*:

        .. math::

            A\\mathbf{a} = \\mathbf{0}

        and

        .. math::

            A^{t}\\mathbf{a} = \\mathbf{0}

        Its form is:

            ::

                    [0  -a3  a2]
                A = [a3  0  -a1]
                    [-a2 a1   0]

        This matrix is usually denoted as :math:`[\\mathbf{a}]_x`.

        :param a: Vector

        .. math::

            \left(\\begin{matrix} a_1 & a_2 & a_3 \\end{matrix}\\right)^t

        :type a: Numpy 1x3 ndarray
        :returns: The 3x3 skew-symmetric matrix associated with
                  :math:`\\mathbf{a}`.
        :rtype: Numpy 3x3 ndarray

        """
        return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])

    def P_from_F(self, F=None):
        """ Computes the second camera matrix (*current frame*) from the
        Fundamental matrix. Assuming the canonical form of camera matrices, i.e,
        the first matrix is of the simple form :math:`[I|\\mathbf{0}]`, where
        :math:`I` is the 3x3 identity matrix and :math:`\\mathbf{0}` a null
        3-vector, the second camera matrix :math:`P'` can be computed as
        follows:

        .. math::

            P' = [[\\mathbf{e'}]_x F|\\mathbf{e'}]

        Recall that we can only recover the camera matrix :math:`P'` up to a
        projective transformation. This means that the mapping between the
        Fundamental matrix :math:`F` and the pair of camera matrices :math:`P`,
        :math:`P'` **is not injective (one-to-one)**. See HZ_ chapter 9 for more
        information about this.

        :param F: Fundamental matrix. If None, then use the internal F
                  parameter.
        :type F: Numpy 3x3 ndarray

        :returns: The computed second camera matrix :math:`P'`.
        :rtype: Numpy 3x4 ndarray.


        """
        if F is None:
            F = self.F
        #e = self.get_epipole(F.T)  # Left epipole
        U, S, V = linalg.svd(F.T)
        e = V[-1]
        e = e / e[2]

        skew_e = self.skew(e)
        return (np.vstack((np.dot(skew_e, F.T).T, e)).T)

    def create_P1(self):
        """ Create a camera matrix of the form:

            ::

                    [1  0  0  0]
                P = [0  1  0  0]
                    [0  0  1  0]

        :returns: Camera matrix with no rotation and no translation components.
        :rtype: Numpy 3x4 ndarray
        """
        P1 = (np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]))
        return P1.astype(float)

    def optimal_triangulation(self, kpts1, kpts2, P1=None, P2=None, F=None):
        """This method computes the structure of the scene given the image
        coordinates of a 3D point :math:`\\mathbf{X}` in two views and the
        camera matrices of those views.

        As Hartley and Zisserman said in their book (HZ_), *naive triangulation
        by back-projecting rays from measured image points will fail, because
        the rays will not intersect in general, due to errors in the measured
        image coordinates*. In order to triangulate properly the image points
        it is necessary to estimate a best solution for the point in
        :math:`\\mathbb{R}^3`.

        The method proposed in HZ_, which is **projective-invariant**, consists
        in estimate a 3D point :math:`\\hat{\\mathbf{X}}` which exactly
        satisfies the supplied camera geometry (i.e, the given camera matrices),
        so it projects as

        .. math::

            \\hat{\\mathbf{x}} = P\\hat{\\mathbf{X}}

        .. math::

            \\hat{\\mathbf{x}}' = P'\\hat{\\mathbf{X}}

        and the aim is to estimate :math:`\\hat{\\mathbf{X}}` from the image
        measurements :math:`\\mathbf{x}` and :math:`\\mathbf{x}'`. The MLE,
        under the assumption of Gaussian noise is given by the point
        :math:`\\hat{\\mathbf{X}}` that minimizes the **reprojection error**

        .. math::

            \\epsilon(\\mathbf{x}, \\mathbf{x}') = d(\\mathbf{x},
                                          \\hat{\\mathbf{x}})^2 + d(\\mathbf{x}'
                                           ,\\hat{\\mathbf{x}}')^2

        subject to

        .. math::

            \\hat{\\mathbf{x}}'^TF\\hat{\\mathbf{x}} = 0

        where :math:`d(*,*)` is the Euclidean distance between the points.

        .. image:: ../Images/triangulation.png

        So, the proposed algorithm by Hartley and Zisserman in their book is
        first to find the corrected image points :math:`\\hat{\\mathbf{x}}` and
        :math:`\\hat{\\mathbf{x}}'` minimizing :math:`\\epsilon(\\mathbf{x},
        \\mathbf{x}')` and then compute :math:`\\hat{\\mathbf{X}}'` using the
        DLT triangulation method (see HZ_ chapter 12).

        :param kpts1: Measured image points in the first image,
                      :math:`\\mathbf{x}`.
        :param kpts2: Measured image points in the second image,
                      :math:`\\mathbf{x}'`.
        :param P1: First camera, :math:`P`.
        :param P2: Second camera, :math:`P'`.
        :param F: Fundamental matrix.
        :type kpts1: Numpy nx2 ndarray
        :type kpts2: Numpy nx2 ndarray
        :type P1: Numpy 3x4 ndarray
        :type P2: Numpy 3x4 ndarray
        :type F: Numpy 3x3 ndarray

        :returns: The two view scene structure :math:`\\hat{\\mathbf{X}}` and
                  the corrected image points :math:`\\hat{\\mathbf{x}}` and
                  :math:`\\hat{\\mathbf{x}}'`.
        :rtype: * :math:`\\hat{\\mathbf{X}}` :math:`\\rightarrow`  Numpy nx3 ndarray
                * :math:`\\hat{\\mathbf{x}}` and :math:`\\hat{\\mathbf{x}}'`
                  :math:`\\rightarrow` Numpy nx2 ndarray.

        """

        kpts1 = np.float32(kpts1)  # Points in the first camera
        kpts2 = np.float32(kpts2)  # Points in the second camera

        # 3D Matrix : [kpts1[0] kpts[1]... kpts[n]]

        pt1 = np.reshape(kpts1, (1, len(kpts1), 2))

        pt2 = np.reshape(kpts2, (1, len(kpts2), 2))

        new_points1, new_points2 = cv2.correctMatches(self.F, pt2, pt1)

        self.correctedkpts1 = new_points1
        self.correctedkpts2 = new_points2

        # Transform to a 2D Matrix: 2xn

        kpts1 = (np.reshape(new_points1, (len(kpts1), 2))).T
        kpts2 = (np.reshape(new_points2, (len(kpts2), 2))).T

        points3D = cv2.triangulatePoints(self.cam1.P, self.cam2.P, kpts2, kpts1)

        #self.structure = points3D / points3D[3]  # Normalize points [x, y, z, 1]
        self.structure = []
        self.structure = cv2.convertPointsFromHomogeneous(np.float32(points3D.T))

        # array = np.zeros((4, len(self.structure[0])))
        #
        # for i in range(len(self.structure[0])):
        #
        #     array[:, i] = self.structure[:, i]
        #
        # self.structure = array

        # The individual points are selected like these:

        # self.structure[:, i]. It's a 4 x n matrix

    def opt_triangulation(self, x1, x2, P1, P2):
        # For each given point corresondence points1[i] <-> points2[i], and a
        # fundamental matrix F, computes the corrected correspondences
        # new_points1[i] <-> new_points2[i] that minimize the geometric error
        # d(points1[i], new_points1[i])^2 + d(points2[i], new_points2[i])^2,
        # subject to the epipolar constraint new_points2^t * F * new_points1 = 0
        # Here we are using the OpenCV's function CorrectMatches.

        # @param x1: points in the first camera, list of vectors x, y
        # @param x2: points in the second camera
        # @param P1: Projection matrix of the first camera
        # @param P2: Projection matrix of the second camera
        # @return points3d: Structure of the scene, 3 x n matrix

        x1 = np.float32(x1)  # Imhomogeneous
        x2 = np.float32(x2)

        # 3D Matrix : [kpts1[0] kpts[1]... kpts[n]]

        x1 = np.reshape(x1, (1, len(x1), 2))

        x2 = np.reshape(x2, (1, len(x2), 2))

        self.correctedkpts1, self.correctedkpts2 = cv2.correctMatches(self.F,
                                                                      x1, x2)
        # Now, reshape to n x 2 shape
        self.correctedkpts1 = self.correctedkpts1[0]
        self.correctedkpts2 = self.correctedkpts2[0]
        # and make homogeneous
        x1 = self.make_homog(np.transpose(self.correctedkpts1))
        x2 = self.make_homog(np.transpose(self.correctedkpts2))

        # Triangulate
        # This function needs as arguments the coordinates of the keypoints
        # (form 3 x n) and the projection matrices

        points3d = self.triangulate_list(x1, x2, P2, P1)

        self.structure = points3d  # 3 x n matrix

        cv2.convertPointsFromHomogeneous(np.float32(points3D.T))

        return points3d

    def triangulate_point(self, x1, x2, P2, P1):
        # Point pair triangulation from least squares solution
        M = np.zeros((6, 6))
        M[:3, :4] = P1
        M[3:, :4] = P2
        M[:3, 4] = -x1
        M[3:, 5] = -x2

        U, S, V = linalg.svd(M)
        X = V[-1, :4]

        return X / X[3]

    def triangulate_list(self, x1, x2, P1, P2):
        # Two view triangulation of points in homogeneous coordinates (several)

        n = x1.shape[1]
        if x2.shape[1] != n:
            raise ValueError("Number of points don't match")

        X = [self.triangulate_point(x1[:, i],
                                    x2[:, i], P1, P2) for i in range(n)]
        return np.array(X).T

    def make_homog(self, points):
        """ Convert points to homogeneus form.

        This method appends one row (fill of ones) to the passed matrix.

        :param points: Matrix of points (2D or 3D) in column form, i.e,
                       the shape of the matrix must be (2 or 3, n), where
                       n is the number of points.
        :type points: Numpy ndarray

        """
        return np.vstack((points, np.ones((1, points.shape[1]))))

    def triangulate(self, kpts1, kpts2, F=None, euclidean=False):
        """ Triangulate 3D points from image points in two views.

        This is the linear triangulation method, which is not an optimal method.
        See chapter 12 of HZ_ for more details.

        If the *euclidean* parameter is True, then the method reconstructs the
        scene up to a similarity transformation. In order to achieve this, it
        computes internally the Essential matrix from the Fundamental one,
        recover the Pose :math:`[R|\\mathbf{t}]` and form the camera projection
        matrix :math:`P'` as

        .. math::

            P' = K[R|\\mathbf{t}]

        The first camera matrix is also multiplied by the camera
        calibration matrix:

        .. math::

            P = K[I|\\mathbf{0}]

        Otherwise, the camera matrices are computed as:

        .. math::

            P' = [[\\mathbf{e'}]_xF|\\mathbf{e}']

        .. math::

            P = [I|\\mathbf{0}]

        and the reconstruction is up to an arbitrary projective transformation.

        .. note::

            If we are performing a reconstruction up to a similarity
            transformation we can filter out those points that don't pass the
            cheirality check by removing the 3D points
            :math:`\\mathbf{X}_i` for which the :math:`Z` coordinate is negative
            (i.e, those points that are projected behind the camera).

            If the reconstruction is up to a projective transformation then it's
            possible that all the triangulated points are behind the camera, so
            don't care about this.

        .. note::

            The method sets the rotation matrix :math:`R` and translation
            vector :math:`\\mathbf{t}` of the internal camera object (**which is
            associated with the second frame**).

        The method normalize the calculated 3D points :math:`\\mathbf{X}`
        internally.

        :param kpts1: Image points for the first frame, :math:`\\mathbf{x}`
        :param kpts2: Image points for the second frame, :math:`\\mathbf{x}'`
        :param F: Fundamental matrix
        :param Euclidean: If True, reconstruct structure up to an Euclidean
                          transformation (using the Essential matrix). Else,
                          reconstruct up to a projective transformation.
        :type kpts1: Numpy nx2 ndarray
        :type kpts2: Numpy nx2 ndarray
        :type F: Numpy 3x3 ndarray
        :type euclidean: Boolean

        :returns: Triangulated 3D points, :math:`\\mathbf{X}` (homogeneous)
        :rtype: Numpy nx4 ndarray

        """
        if np.shape(kpts1)[1] != 2:
            raise ValueError("The dimensions of the input image points must \
                              be (n, 2), where n is the number of points")
        #print ("Shape needed for recoverpose: {}".format(np.shape(kpts1)))
        #print ("Type needed for recoverpose: {}".format(type(kpts1)))
        #print ("Type: {}".format(type(kpts1[0][0])))
        kpts1 = (np.reshape(kpts1, (len(kpts1), 2))).T
        kpts2 = (np.reshape(kpts2, (len(kpts2), 2))).T
        if F is None:
            F = self.F
        if euclidean:
            E = self.E_from_F(F)
            R, t = self.get_pose(kpts1.T, kpts2.T, self.cam.K, E)
            self.cam.set_R(R)
            self.cam.set_t(t)
            P2 = self.cam.Rt2P(R, t, self.cam.K)
            P1 = np.dot(self.cam.K,
                        np.array([[1, 0, 0, 0],
                                  [0, 1, 0, 0],
                                  [0, 0, 1, 0]]))
        else:
            P2 = self.P_from_F()
            P1 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
        points3D = cv2.triangulatePoints(P1, P2, kpts1, kpts2)
        euclideanPoints = []
        euclideanPoints = cv2.convertPointsFromHomogeneous(np.float32(points3D.T))
        return euclideanPoints

    def filter_z(self, points):
        """ Filter out those 3D points whose Z coordinate is negative and is
        likely to be an outlier, based on the median absolute deviation (MAD).

        The mask returned by the method can be used to filter the image points
        in both images, :math:`\\mathbf{x}`  and :math:`\\mathbf{x}'`.

        :param points: 3D points :math:`\\mathbf{X}`
        :type points: Numpy nx4 ndarray

        :returns: 1. 3D points filtered
                  2. Filter mask (positive depth and no outliers)
        :rtype: 1. Numpy nx4 ndarray
                2. Numpy 1xn ndarray

        """
        points = np.reshape(points,(points.shape[0],3))
        mask_pos = points[:, 2] >= 0
        thresh = 3.5
        Z = points[:, 2]
        Z = Z[:, None]
        median = np.median(Z, axis=0)
        diff = np.sum((Z-median)**2, axis=1)
        diff = np.sqrt(diff)
        med_abs_deviation = np.median(diff)
        modified_z_score = 0.6745 * diff / med_abs_deviation
        mask = modified_z_score < thresh
        return points[mask & mask_pos], mask & mask_pos

    def reprojection_error(self, KRt, points3d, kps):
        # Reprojection of the 3D points to 2D points on the image.
        # A large reprojection distance may indicate an error in triangulation,
        # so we may not want to include this point in the final result.
        # High average reprojection rates may point to a problem with
        # the P matrices, and therefore a possible problem with
        # the calculation of the essential matrix or the matched feature points.
        K = np.reshape(KRt[0:9],(3,3))
        Rvector = KRt[9:12]
        t = KRt[12:15]
        # reproject points
        imgpoints, jacobian = cv2.projectPoints(points3d, Rvector, t, K, distCoeffs = None)
        # check individual reprojection error
        diff =  imgpoints - np.reshape(imgpoints,kps.shape)
        #err = np.linalg.norm(diff, axis = 0) # vector of errors
        return diff.ravel()


    def convert_from_homogeneous(self, kpts):
        # Convert homogeneous points to euclidean points
        # @param kpts: List of homogeneous points
        # @return pnh: list of euclidean points

        # Remember that every function in OpenCV need us to specify the data
        # type. In addition, convertPointsFromHomogeneous needs the shape of the
        # arrays to be correct. The function takes a vector of points in c++
        # (ie. a list of several points), so in numpy we need a multidimensional
        # array: a x b x c where a is the number of points, b=1, and c=2 to
        # represent 1x2 point data.

        if len(kpts[0]) == 3:

            for i in range(len(kpts)):

                kpts[i].reshape(-1, 1, 3)

                kpts[i] = np.array(kpts[i], np.float32).reshape(-1, 1, 3)

            pnh = [cv2.convertPointsFromHomogeneous(x) for x in kpts]

            for i in range(len(pnh)):

                pnh = np.array(pnh[i], np.float32).reshape(1, 2, 1)

        elif len(kpts[0]) == 4:

            for i in range(len(kpts)):

                kpts[i].reshape(-1, 1, 4)

                kpts[i] = np.array(kpts[i], np.float32).reshape(-1, 1, 4)

            pnh = [cv2.convertPointsFromHomogeneous(x) for x in kpts]

            for i in range(len(pnh)):

                pnh[i] = np.array(pnh[i], np.float32).reshape(1, 3, 1)

        elif len(kpts) == 3:

            pnh = np.zeros((2, len(kpts[0])))

            for i in range(len(kpts[0])):

                pnh[:, i] = kpts[:2, i]

        return pnh

    def convert_array2d(self, kpts):

        print len(kpts[:, 0])

        a = np.zeros((len(kpts[:, 0]), 2))

        for i in range(len(kpts[:, 0])):

            a[i, :] = kpts[i, :2]

        return a

    def optimal_solution(self, F, matches):

        for match in matches:

            (x1,y1) = self.kp1[match.queryIdx].pt
            (x2,y2) = self.kp2[match.trainIdx].pt
            # We create the traslation matrix that put the keypoints in
            # the origin
            T1 = np.matrix('1 0 -x1; 0 1 -y1; 0 0 1')
            T2 = np.matrix('1 0 -x2; 0 1 -y2; 0 0 1')
            # Translate F
            # note that if x'^T F x = 0,
            # and if x_t = Tx
            # Then
            # T^(-1T) F T^(-1) = 0
            Ft = np.linalg.inv(T1).T*F*np.linalg.inv(T2)
            # Calculate the epipolar points
            # F*e = 0
            # This problem has  the form Ax = 0: its an homogeneous equation
            u,s,vt = np.linalg.svd(Ft,full_matrices=True)
            e1 = vt[:,2]
            e1 = cv2.convertPointsFromHomogeneous(e1.T)
            # e'^T*F = 0
            # To give i t the form Ax=0, we transpose to change the form
            # F^Te' = 0
            u,s,vt = np.linalg.svd(Ft.T,full_matrices=True)
            e2 = vt[:,2]
            e1 = cv2.convertPointsFromHomogeneous(e2.T)
            # Form the rotation matrices
            R1 = np.matrix('e1[0] e1[1] 0; -e1[1] e1[0] 0; 0 0 1')
            R2 = np.matrix('e2[0] e2[1] 0; -e2[1] e2[0] 0; 0 0 1')

            # Replace R by R'FR^T
            Ftr = R2*F*R1.T

            # Form the polynomial
            f1 = e1[2]
            f2 = e2[2]
            a = Ftr[2][2]
            b = Ftr[2][3]
            c = Ftr[3][2]
            d = Ftr[3][3]
            g = x*((a*x+b)^2 + f2^2*(c*x + d)^2 - (a*d-b*c)*(1+f1^2*x^2)^2*(a*x+b)*(c*x+d))






    def func(self, params, x1, x2):
        """ Computes the residuals for the Fundamental matrix two view
        optimization problem.

        This is an m-dimensional function of n variables (n is the number of
        observations in the frames, the image points, and m in this case is
        :math:`2n`) that returns the residuals between the measured image points
        and the projections of  the reconstructed 3D points
        :math:`\\hat{\\mathbf{X}}`: :math:`\\hat{\\mathbf{x}}`
        and :math:`\\hat{\\mathbf{x}}'`.

        The method compute the projected points :math:`\\hat{\\mathbf{x}}` and
        :math:`\\hat{\\mathbf{x}}'` from the two camera projection matrices,
        :math:`P` (created using the
        :py:mod:`VisualOdometry.VisualOdometry.create_P1` method)
        and :math:`P'`, which is extracted from the parameters vector (the
        first twelve elements).


        :param params: Parameter vector :math:`\\mathbf{p}`, that contains the
                       second camera parameters and the 3D structure.
        :param x1: The first frame measured points :math:`\\mathbf{x}`
        :param x2: The second frame measured points :math:`\\mathbf{x}'`
        :type params: Numpy ndarray of shape :math:`k`, where :math:`k` is the
                      sum of the second camera parameters and the 3D parameters.
        :type x1: Numpy nx2 ndarray
        :type x2: Numpy nx2 ndarray


        """
        P1 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
        P2 = params[0:12].reshape(3, 4)
        p = params[12:len(params)]
        l = p.shape
        X = np.reshape(p, (l[0] / 3, 3)).T # 3xn matrix
        # Make homogeneous by appending a row of ones
        X = np.vstack((X, np.ones((1, X.shape[1]))))

        # Project the structure
        x1_est = np.dot(P1, X)
        # Convert from homogeneus
        x1_est = (x1_est/x1_est[2])[:2, :]
        # Same with x2
        x2_est = np.dot(P2, X)
        x2_est = (x2_est/x2_est[2])[:2, :]


        error_image1 = (x1 - x1_est.T).ravel()
        error_image2 = (x2 - x2_est.T).ravel()
        error = np.append(error_image1, error_image2)
        return error

    def optimize_F(self, x1, x2, F=None, structure=None,
                   method='lm', robust_cost_f='linear'):
        """ Minimize the cost

        .. math::

            \\epsilon(\\mathbf{x}, \\mathbf{x}') = \\sum_i d(\\mathbf{x}_i,
            \\hat{\\mathbf{x}}_i)^2 +
            d(\\mathbf{x}_i', \\hat{\\mathbf{x}}_i')^2

        over an initial estimate of :math:`\\hat{F}` and
        :math:`\\hat{\\mathbf{X}}_i`, :math:`i=1,\\dots, n`

        The cost is minimized using a nonlinear minimization algorithm over
        :math:`3n+12` variables: :math:`3n` for the 3D points
        :math:`\\hat{\\mathbf{X}}_i` and 12 for the camera matrix
        :math:`P'=[M|\\mathbf{t}]`, with :math:`\\hat{F}=[\\mathbf{t}]_xM` and

        .. math::

            \\hat{\\mathbf{x}}_i = P\\mathbf{x}_i

            \\hat{\\mathbf{x}}_i' = P'\\mathbf{x}_i'

        The available algorithms are:

            * **trf**: Trust Region Reflective algorithm, see :cite:`branch1999`
            * **dogbox**: Modified Powell's Dogleg algorithm, see
              :cite:`powell1970new` and :cite:`voglisrectangular`.
            * **lm**: Levenberg-Marquardt algorithm, see
              :cite:`more1978levenberg`.

        In order to reduce the influence of outliers on the solution we can
        modify the cost function :math:`\\epsilon(\\mathbf{x}, \\mathbf{x}')`
        using the robust_cost_f argument:

            * **linear**: Standard least-squares (no modification)
            * **soft_l1**: Pseudo-Huber cost function:

                .. math::

                    C(\\epsilon) = 2\\sqrt{1*\\epsilon}-1

            * **Huber**: Huber cost function:

                .. math::

                    C(\\epsilon) = \\epsilon \\ \\mathbf{if \\ \\epsilon\\leq 1}

                    \\mathbf{else} \\ C(\\epsilon) = 2\\sqrt{\\epsilon}

            * **Cauchy**: Cauchy cost function:

                .. math::

                    C(\\epsilon) = ln(1+\\epsilon)

        .. warning::

            If we are using the Levenberg-Marquardt algorithm the cost function
            must be the **linear** one. Otherwise the algorithm will raise an
            error.

        :param x1: The previous frame measured image points, :math:`\\mathbf{x}`
        :param x2: The current frame measured image points, :math:`\\mathbf{x}'`
        :param F: Fundamental matrix. If None, then the internal attribute will
                  be used.
        :param structure: 3D scene structure, :math:`\\hat{\\mathbf{X}}`
        :param method: Minimization algorithm to be used.
        :param robust_cost_f: Robust cost function to be used.
        :type x1: Numpy nx2 ndarray
        :type x2: Numpy nx2 ndarray
        :type F: Numpy 3x3 ndarray
        :type structure: Numpy nx4 Numpy ndarray
        :type method: String
        :type robust_cost_f: String

        :returns: 1. Instance of the scipy.optimize.OptimizeResult (contains
                     all the information returned by the minimization algorithm)

                  2. Optimized Fundamental matrix.

        :rtype:

                1. :math:`F`: Numpy 3x3 ndarray
                2. :py:mod:`scipy.optimize.OptimizeResult` instance.


        """
        if F is None:
            F = self.F
        vec_P2 = np.hstack(self.P_from_F())
        # Transform the structure (matrix 3 x n) to 1d vector
        if structure is None:
            structure = self.structure
        vec_str = structure.reshape(-1)
        param = vec_P2
        param = np.append(param, vec_str)
        solution = optimize.least_squares(self.func, param, method=method,
                                          args=(x1, x2), loss=robust_cost_f,
                                          ftol = 1e-3, xtol = 1e-3)
        P = solution.x[:12].reshape((3, 4))
        M = P[:, :3]
        t = P[:, 3]
        F = np.dot(self.skew(t), M)
        return solution, F

    def mask_reprojection(self, K,R,t, points3D, x):
        # Reprojection of the 3D points to 2D points on the image.
        # A large reprojection distance may indicate an error in triangulation,
        # so we may not want to include this point in the final result.
        # High average reprojection rates may point to a problem with
        # the P matrices, and therefore a possible problem with
        # the calculation of the essential matrix or the matched feature points.
        Rvector,_ = cv2.Rodrigues(R)
        # reproject points
        imgpoints, jacobian = cv2.projectPoints(points3D, Rvector, t, K, distCoeffs = None)
        # check individual reprojection error
        diff =  x - np.reshape(imgpoints,x.shape)
        print diff

        mask = [np.linalg.norm(res,axis = 0) < 0.1 for res in diff]
        return mask

    def E_from_F(self, F=None, K=None):
        """ This method computes the Essential matrix from the Fundamental
        matrix.

        The equation is the following:

        .. math::

            E = K^{t}FK

        where :math:`K` is the camera calibration matrix, a 3x3 matrix that
        contains the intrinsic parameters of the camera:

        ::

                    [f    px]
            K  =    [  f  py]
                    [     1 ]


        For a detailed discussion about these topics see HZ_ chapters 6 and 9.

        .. _HZ: http://www.robots.ox.ac.uk/~vgg/hzbook/

        :param F: Fundamental matrix. If None, use the internal attribute.
        :type F: Numpy 3x3 ndarray
        :param K: Camera calibration matrix
        :type K: Numpy 3x3 ndarray

        :returns: The estimated Essential matrix E
        :rtype: Numpy ndarray (3x3)

        """
        if F is None:
            F = self.F
        if K is None:
            K = self.cam.K
        self.E = K.transpose().dot(F).dot(K)
        return self.E

    def get_pose(self, pts1, pts2, camera_matrix, E=None, inplace=True):
        """ Recover the rotation matrix :math:`R` and the translation
        vector :math:`\\mathbf{t}` from the Essential matrix.

        As Hartley and Zisserman states in their book, the camera pose can be
        recovered from the Essential matrix up to scale. For a given Essential
        matrix :math:`E`, and first camera matrix :math:`P=[I|\\mathbf{0}]`,
        there are four possible solutions for the second camera matrix
        :math:`P'` (see HZ_ section 9.6.2).

        A reconstructed point :math:`\\mathbf{X}` will be in front of both
        cameras in one of these four solutions only. Thus, testing with a single
        point to determine if it is in front of both cameras is sufficient to
        decide between the four different solutions for the camera :math:`P'`.

        OpenCV 3 has an implementation of the Nister_ five point algorithm to
        extract the pose from the Essential matrix and a set of corresponding
        image points (KeyPoints). The algorithm follow these steps:

            1. Extract the two possible solutions for the rotation matrix
               :math:`R` and also the two solutions for the translation vector
               :math:`\\mathbf{t}`, so we have the four possible solutions:

               .. math::

                   P_1 = [UWV^T|\\mathbf{t}]

               .. math::

                   P_2 = [UWV^T|-\\mathbf{t}]

               .. math::

                   P_3 = [UW^TV^T|\\mathbf{t}]

               .. math::

                   P_4 = [UW^TV^T|-\\mathbf{t}]


              with :math:`R=UWV^T` or :math:`R=UW^TV^T` and :math:`\\mathbf{t}`
              being the last column of :math:`U`.

            2. For all the four possible solutions do:

                2.1. Triangulate the set of corresponding
                     KeyPoints and normalize them, i.e, divide all the
                     vector elements by its fourth coordinate
                     (we are working with **homogeneous**  coordinates here):

                     .. math::

                         for \\ every \\ 3D \\ triangulated \\ point \\ \\mathbf{X}_i:



                         \\mathbf{X}_i = \\frac{\\mathbf{X}_i}{\\mathbf{X}_i^3}

                3.1. Next, Nister uses a threshold distance to filter out far
                     away points (i.e, points at infinity). Then, the algorithm
                     filter those triangulated points that have the third
                     coordinate (depth) less than zero and count the number of
                     them that meet these constraints (the valid points)

            4. The solution that have more valid triangulated points is the
               true one.

        .. note::

                In order to compute the pose of the second frame with respect
                to the first one we invert the order of the parameters *pts* and
                *pts2* when passing them to the OpenCV method recoverPose.

        :param E: Essential matrix, if None then used the internal one.
        :param pts1: Points from the first image
        :param pts2: Points from the second image
        :param camera_matrix: Camera calibration matrix
        :param inplace: If True, then fill the :math:`R` and :math:`\\mathbf{t}`
                        vectors of the current camera. Also, compute the
                        camera projection matrix :math:`P` **up to scale**.
        :type E: Numpy 3x3 ndarray
        :type pts1: Numpy nx2 ndarray
        :type pts2: Numpy nx2 ndarray
        :type camera_matrix: Numpy 3x3 ndarray

        :returns: The rotation matrix :math:`R`, the translation vector and
                  a mask vector with the points that have passed the cheirality
                  check.
        :rtype: Numpy ndarrays

        """
        if E is None:
            E = self.E
        R = np.zeros([3, 3])
        t = np.zeros([3, 1])
        pp = tuple(camera_matrix[:2, 2])
        f = camera_matrix[0, 0]
        #pts1 = pts1.astype(np.float64)
        #pts2 = pts2.astype(np.float64)
        cv2.recoverPose(E, pts2, pts1, R, t, f, pp, mask = None)
        if inplace:
            self.cam.set_R(R)
            self.cam.set_t(t)
            self.cam.set_P(self.cam.Rt2P(R, t, self.cam.K))
        return R, t

    def compute_scale(self, plane_model, scene):
        # Compute the scale of the scene based on a plane fitted to the 3D point
        # cloud represented by scene. The plane_model is fitted using a
        # least-squares approach inside a RANSAC scheme (see PlaneModel.py)
        # @param plane_model: the parameters of the plane (numpy array)
        # @param scene: 3D points marked as inliers by the RANSAC algorithm in
        # the process of estimating the plane. (4 x n) numpy array
        # @return scale: scale of the scene (float)

        # First compute the distance for every inlier and take the mean as the
        # final distance
        distance_sum = 0
        for i in range(np.shape(scene)[1]):
            distance = (np.dot(plane_model, scene[:, i])) / \
                        np.linalg.norm(plane_model)
            distance_sum += distance
        # Compute the mean distance and the corresponding scale as H / d
        mean = distance_sum / np.shape(scene)[1]
        scale = self.height / mean

        return scale

    def compute_scale2(self,  scene, pitch=0):
        # Compute the scale using an heuristic approach. For every triangulated
        # point compute it's height and the height difference with respect to
        # the other points. Sum all this differences and use a heuristic
        # function to decide which height is selected
        # @param pitch: The pitch angle of the camera (by default zero)
        # @param scene: 3D points of the hypothetical ground plane (4 x n)
        max_sum = 0
        for i in range(np.shape(scene)[1]):
            h = scene[1][i] * cos(pitch) - scene[2][i] * sin(pitch)
            height_sum = 0
            for j in range(np.shape(scene)[1]):
                h_j = scene[1][j] * cos(pitch) - scene[2][j] * sin(pitch)
                height_diff = h_j - h
                height_sum += exp(-50 * height_diff * height_diff)
            if height_sum > max_sum:
                max_sum = height_sum
                best_idx = i
        scale = scene[1][best_idx] * cos(pitch) - \
                scene[2][best_idx] * sin(pitch)
        return scale

"""
.. bibliography:: zreferences.bib
    :all:
"""
