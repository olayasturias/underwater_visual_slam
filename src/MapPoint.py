from scipy.spatial import distance
import numpy as np

"""
.. codeauthor:: Cesar Gonzalez Gonzalez
:file MapPoint.py
"""


class MapPoint(object):
    """ The MapPoint class represents a 3D world point.

    .. _ORB-SLAM: https://arxiv.org/pdf/1502.00956.pdf
    .. _Hamming: https://en.wikipedia.org/wiki/Hamming_distance

    **Attributes**:

        .. data:: x_w

           Its 3D position :math:`\\mathbf{X}_{w,i}` in the *world* coordinate
           system.

        .. data:: ORB_descriptor

           Representative ORB descriptor :math:`D`, which is the associated
           ORB descriptor whose hamming distance is minimum with respect to all
           other associated descriptors in the keyframes in which the point is
           observed (see ORB-SLAM_).

        .. data:: descriptors

           Numpy ndarray  of descriptors associated with the image projection
           of the point in each frame. As the
           :py:class:`MapPoint.measured_2d_points`, this ndarray is correlated
           with the connected_frames list.

           Each entry of this ndarray is a numpy 1xd ndarray, where :math:`d` is
           the size of the descriptor (:math:`d=32` for ORB descriptors).

        .. data:: connected_frames

           List of Camera objects in which the point was observed.

        .. data:: measured_2d_points

           Numpy ndarray of image KeyPoints for the 3D point. This array is
           correlated with the *connected_frames* attribute, i.e, the first
           KeyPoint is the 3D point as seen by the camera wich index is the
           first index in the *connected_frames* list.

        .. data:: index

           List of camera indices. This is an **unordered list**.

    **Constructor**:

        The constructor can take 4 optional arguments:

            1. *coord* (Numpy 1x4 ndarray): The 3D coordinates of the point
               (Homogeneous form).
            2. *frame* (:py:class:`Camera.Camera`): Cameras from which the
               point has been seen. It can be a list of Camera objects or a
               single Camera object.
            3. *img_point* (Numpy ndarray): 2D image coordinates of the
               point, as seen by the camera.
            4. *desc* (Numpy ndarray): Descriptor associated with the image
               point.

    """
    def __init__(self, coord=None, frame=None, img_point=None, desc=None):
        """ Constructor

        """
        self.connected_frames = []
        self.index = None
        if coord is not None:
            self.x_w = coord
        else:
            self.x_w = None
        if frame is not None:
            self.connected_frames.extend(frame)
            self.index = [item.index for item in frame]
        if img_point is not None:
            self.measured_2d_points = img_point
        else:
            self.measured_2d_points = None
        if desc is not None:
            self.descriptors = desc
            if desc.shape[0] > 1:
                self.ORB_descriptor = desc[0]
            else:
                self.ORB_descriptor = desc
        else:
            self.descriptors = None
            self.ORB_descriptor = None

    def set_x(self, point_3d):
        """ Sets the 3D position :math:`\\mathbf{X}_{w,i}` of the map point

        :param point_3d: 3D coordinates to be used as point 3D.
        :type point_3d: Numpy 1x3 ndarray
        """
        self.x_w = point_3d

    def get_x(self):
        """ Returns the 3D coordinates of the point

        :returns: 3D coordinates of the point
        :rtype: Numpy 1x3 ndarray

        """
        return self.x_w

    def add_frame(self, frame):
        """ Adds a frame (Camera) (and its index)to the list of connected
        frames (indices).

        .. warning::

            This method assumes that the Camera object has its index parameter
            set.

        :param frame: New camera object
        :type frame: :py:class:`Camera.Camera` object

        """
        self.connected_frames.append(frame)
        self.index.append(frame.index)

    def remove_frame(self, frame):
        """ Removes a frame index from the connected frames list and the
        2D coordinates and descriptor related with it.

        This method removes all occurrences of the frame index.

        :param frame: The frame index
        :type frame: Integer

        """
        del self.connected_frames[frame]
        del self.index[frame]
        np.delete(self.descriptors, frame, 0)
        np.delete(self.measured_2d_points, frame, 0)

    def add_image_point(self, point):
        """ Add a new projection of the map point in a frame.

        :param point: The 2D coordinates of the image point.
        :type point: Numpy 1x2 ndarray

        """
        self.measured_2d_points = np.vstack((self.measured_2d_points,
                                             point))

    def add_descriptor(self, desc):
        """ Adds a descriptor to the map point

        :param desc: Descriptor
        :type desc: Numpy ndarray (the shape will depend on the type of the
                    descriptor)
        """
        self.descriptors = np.vstack((self.descriptors,
                                      desc))

    def add_observation(self, frame, point, desc):
        """ Adds a complete observation of the map point

        :param frame: The new camera.
        :param point: 2D coordinates in the image plane
        :param desc: Associated descriptor
        :type frame: :py:class:`Camera.Camera` object
        :type point: Numpy 1x2 ndarray
        :type desc: Numpy ndarray
        """
        self.add_frame(frame)
        self.add_image_point(point)
        self.add_descriptor(desc)

    def compute_rep_desc(self):
        """ Computes the representative ORB descriptor of the map point.

        The representative ORB descriptor is computed using the following
        equation:

        .. math::

            D = \\min_i \\sum_j d_H(d_i, d_j)

        where :math:`d_H(d_i, d_j)` is the Hamming_ distance between the
        i-th descriptor and the j-th descriptor.

        """
        hamming_sum = 0
        hamming_list = []
        for i in range(len(self.descriptors)):
            for j in range(len(self.descriptors)):
                if j != i:
                    hamming_sum += distance.hamming(self.descriptors[i],
                                                    self.descriptors[j])
                else:
                    pass
            hamming_list.append(hamming_sum)
        index = hamming_list.index(min(hamming_list))
        self.ORB_descriptor = self.descriptors[index]

    def get_cam(self, index):
        """ Returns the camera whose internal index is equal to index

        :param index: The camera index
        :type index: Integer

        :returns: The desired camera if exist in the list or None otherwise.
        :rtype: :py:class:`Camera.Camera` object

        """
        try:
            id = self.index.index(index)
        except ValueError:
            return None
        return self.connected_frames[id]

    def project_in_list(self, cameras):
        """ Projects the MapPoint into a list of cameras.

        :param cameras: List of :py:class:`Camera.Camera` objects.
        :type cameras: List
        :returns: The projected point for each camera, a Numpy nx3 ndarray,
                  where :math:`n` is the number of cameras
        :rtype: Numpy nx3 ndarray.
        """
        proj_points = cameras[0].project(self.x_w)
        for cam in cameras:
            if cam.index == 0:
                pass
            else:
                np.vstack((proj_points, cam.project(self.x_w)))
        return proj_points

    def project_point(self, index, camera=None):
        """ Projects the MapPoint into a camera frame indexed by index.

        :param index: Camera index
        :param camera: If not None, then project using this Camera
        :type index: Integer
        :type camera: :py:mod:`Camera.Camera` object
        :returns: * Image coordinates of the MapPoint **if the point has been
                    observed in the camera**.
                  * None if the point hasn't been observed in that particular
                    camera.
        :rtype: Numpy (3, ) ndarray

        :todo: This method should have the ability to project
               a map point in a frame other than the ones in 
               which it has been observed, in order to perform
               tracking. Maybe using additional arguments (camera argument)
               and if camera is None then use this camera to project the 
               point.
        """

        if camera is None:
            if index in self.index:
                print ("Index: {}".format(index))
                return self.connected_frames[index].project(self.x_w)
            elif camera is None:
                print ("Index: {}".format(index))
                print ("Point index: {}".format(self.index))
                return None
        else:
            return camera.project(self.x_w)
        

    def ret_if_seen(self, camera_index):
        """ Return the MapPoint if it was observed in the camera indexed by
        the camera_index value.

        :param camera_index: The camera index
        :type camera_index: Integer
        :returns: True if it was observed
        :rtype: Boolean
        """
        if camera_index in self.index:
            return True
        else:
            return False
