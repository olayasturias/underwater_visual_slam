import numpy as np

"""
.. codeauthor:: Cesar Gonzalez Gonzalez
:file Camera.py

"""


class Camera(object):
    """ The Camera class represents the real camera. It has its representative
    attributes and methods. Also, it stores all the camera poses over the time.

    **Attributes**:

        .. data:: K

            Camera calibration matrix, of the form:

                ::

                        [f    px]
                    K = [  f  py]
                        [      1]

            where :math:`f` is the focal distance and :math:`p_x` , :math:`p_y`
            are the camera center coordinates.

            For the Kitti dataset:

                * :math:`f \\ = \\ 7.18856e+02`
                * :math:`p_x \\ = \\ 6.071928e+02`
                * :math:`p_y \\ = \\ 1.852157e+02`

            **Type**: Numpy 3x3 ndarray

        .. data:: P

            The *homogeneous camera projection matrix*, which has shape 3x4:

            .. math::

                P = K[R | \\vec{t}]

            where :math:`R` is a 3x3 rotation matrix and :math:`\\vec{t}` is
            the translation vector between the world and the camera coordinate
            frames.

            **Type**: Numpy 3x4 ndarray

        .. data:: R

            Rotation matrix

            **Type**: Numpy 3x3 ndarray

        .. data:: t

            Translation vector

            **Type**: Numpy 1x3 ndarray

        .. data:: h

            Height of the camera above the ground plane. This parameter can be
            used to compute the scale of the reconstruction.

            **Type**: Float

        .. data:: points

           All the ORB features extracted in the frame, associated or not to a
           map point.

           **Type:** Numpy  ndarray

        .. data:: descriptors

           All the descriptors extracted for the KeyPoints in the frame.

           **Type**: Numpy ndarray

        .. data:: index

           Index of the camera (Integer)

        .. data:: is_kf

           Boolean, if True the frame is a KeyFrame.

    **Constructor**:

        The constructor take as argument a camera calibration matrix :math:`K`.

    """
    def __init__(self, K=None):
        if K is None:
            self.K = np.array([[7.18856e+02, 0.0, 6.071928e+02],
                               [0.0, 7.18856e+02, 1.852157e+02],
                               [0.0, 0.0, 1.0]])
        else:
            self.set_K(K)
        self.P = None
        self.R = None
        self.t = None
        self.h = 1.65
        self.index = None
        self.is_kf = False
        self.descriptors = None
        self.points = None

    def set_descriptors(self, desc):
        """ Adds the descriptors to the frame

        :param desc: list of descriptors (list of numpy ndarrays)
        :type desc: List
        """
        self.descriptors = desc

    def set_points(self, points):
        """ Adds the points to the frame

        :param points: List of numpy ndarrays
        :type points: List
        """
        self.points = points

    def set_index(self, index):
        """ Sets the camera index

        :param index: Camera index
        :type index: Integer
        """
        self.index = index

    def set_K(self, K):
        """ Sets the calibration matrix

        :param K: New calibration matrix
        :type K: Numpy 3x3 ndarray

        """
        self.K = K

    def set_P(self, P):
        """ Sets the projection matrix

        :param P: New projection matrix
        :type P: Numpy 3x4 ndarray

        """
        self.P = P

    def set_R(self, R):
        """ Sets the  Rotation matrix

        :param R: New rotation matrix
        :type R: Numpy 3x3 ndarray

        """
        self.R = R

    def set_t(self, t):
        """ Sets the Translation vector

        :param t: New translation vector
        :type t: Numpy 1x3 ndarray

        """
        self.t = t

    def project(self, X):
        """ Project points in X and normalize coordinates.

        The equation is:

        .. math::

            \\mathbf{x_i} = P\\mathbf{X_i}

        where :math:`\\mathbf{x_i} = (fX_i \\ + \\ Zp_x, fY_i \\ + \\ Zp_y, Z_i)`
        is the vector that represents the image plane coordinates of the 3D-ith
        point :math:`(X_i, Y_i, Z_i, 1)`.

        After projecting the 3D point we normalize the resultant 2D image point
        by dividing the vector by its Z coordinate.

        :param X: Array with the 3D coordinates, one row per 3D point in
                  homogeneous form.
        :type X: Numpy nx4 ndarray.

        :returns: Projected points in homogeneous form
        :rtype: Numpy nx3 ndarray
        """
        x = np.dot(self.P, X)
        for i in range(3):
            x[i] /= x[2]
        return x

    def Rt2P(self, R=None, t=None, K=None, inplace=False):
        """ Compute the camera projection matrix P given the extrinsic
        parameters R and t using:

        .. math::

            P = K[R|\\mathbf{t}]

        :param R: Rotation matrix
        :param t: Translation vector
        :param K: Calibration matrix
        :param inplace: If True result is set as self P matrix. Otherwise the result
                        is returned as numpy ndarray.
        :type R: Numpy 3x3 ndarray
        :type t: Numpy 1x3 ndarray
        :type K: Numpy 3x3 ndarray
        :type inplace: Boolean

        :returns: The camera projection matrix associated with R and t
        :rtype: Numpy 3x4 ndarray
        """
        if R is None:
            R = self.R
        if t is None:
            t = self.t
        if K is None:
            K = self.K
        pose = np.column_stack((R, t))
        if inplace:
            self.P = np.dot(K, pose)
        else:
            return np.dot(K, pose)

    def get_pp(self):
        """ Returns the principal point of the camera

        The principal point (pp) is the vector whose coordinates are the
        coordinates of the camera center:

        .. image:: ../Images/principal_point.png

        This point can be extracted from the calibration matrix :math:`K`:

        .. math::

            pp = (p_x, p_y)

        being :math:`p_x`, :math:`p_y` the elements (1,3) and (2,3) of the
        calibration matrix, respectively.

        :returns: Principal point of the camera
        :rtype: Numpy 1x2 ndarray

        """
        return self.K[:2, 2]

    def get_focal(self):
        """ Returns the focal lenght of the camera

        For the Kitti dataset the images have the same :math:`f_x` and
        :math:`f_y`, so we simply return the first element of the calibration
        matrix :math:`K`.

        :return: The focal lenght :math:`f` of the camera
        :rtype: Float
        """
        return self.K[0, 0]

    def is_keyframe(self):
        """ Sets the Camera (frame) as a KeyFrame.

        """
        self.is_kf = True
