"""
.. codeauthor:: Cesar Gonzalez Gonzalez
:file Map.py
"""
import numpy as np


class Map(object):
    """ The Map class represents the world map as seen by the camera.

    **Attributes**:

        .. data:: cameras

           List of Camera objects. One Camera per frame.

           .. seealso::

               :py:mod:`Camera.Camera`

        .. data:: structure

           List of 3D points, the structure reconstruction. Each 3D point is
           an instance of the :py:mod:`MapPoint.MapPoint` class.

           .. seealso::

               :py:mod:`MapPoint.MapPoint`

    **Constructor**:

        We can pass to the constructor a camera object (the initial frame) and
        a list of 3D MapPoint objects that represents the structure seen at
        the moment. If don't pass parameters to the constructor, then the
        first Camera (Frame) will be set as the origin (no rotation and no
        translation), and the structure will be an empty list.

    """
    def __init__(self, init_cam=None, init_structure=None):
        """ Constructor

        :param init_cam: The initial camera (optional)
        :param init_structure: The initial structure (optional)
        :type init_cam: :py:class:`Camera.Camera`
        :type init_structure: List of :py:class:`MapPoint` objects.

        """
        self.cameras = []
        self.structure = []
        if init_cam is not None:
            self.add_camera(init_cam)
        if init_structure is not None:
            self.add_list_mappoints(init_structure)

    def add_camera(self, camera):
        """ Add a camera (frame) to the list of cameras in *cameras* attribute.

        :param camera: The new camera (frame).
        :type camera: :py:class:`Camera.Camera` object

        """
        self.cameras.append(camera)

    def add_list_mappoints(self, mappoints):
        """ Add a list of mappoints to the list of mappoints in *structure*
        attribute.

        :param mappoints: The list of mappoints
        :type mappoints: List of :py:class:`MapPoint.MapPoint` objects

        """
        self.structure.extend(mappoints)

    def add_mappoint(self, point):
        """ Add a MapPoint to the stored structure

        :param point: The new map point
        :type point: :py:class:`MapPoint.MapPoint` object

        """
        self.structure.append(point)

    def ret_local_map(self, index):
        """ Returns the local map about the frame index passed.

        This method returns the 3D structure, :math:`\\mathbf{X}_i`, seen by
        the five previous KeyFrames.

        :param index: Index of the frame for which we are quering the local map.
        :type index: Integer
        :returns: Local map.
        :rtype: List of :py:class:`MapPoint.MapPoint` objects.
        """
        local_map = self.ret_local_kf(index)
        local_map_pt = []
        for point in self.structure:
            for camera in local_map:
                if point.ret_if_seen(camera):
                    if point in local_map_pt:
                        pass
                    else:
                        local_map_pt.append(point)
                        break
        return local_map_pt, local_map

    def ret_local_kf(self, id):
        """ Returns the five previous KeyFrames to the frame indexed by id.

        :param id: Index of the frame
        :type id: Integer
        :returns: The five previous KeyFrames
        :rtype: List of indices (integers).
        """
        cameras = self.cameras[:id + 1]
        local_map = [camera.index for camera in cameras if camera.is_kf]
        if len(local_map) < 5:
            return local_map
        else:
            return local_map[id-5:id]

    def project_local_map(self, index):
        """ Projects the local map in the camera indexed by index.

        :param index: Index of the frame in which we are going to project the
                      map.
        :type index: Integer
        :returns: Coordinates of the image points, :math:`\\hat{\\mathbf{x}}`
        :rtype: Numpy nx3 ndarray, where :math:`n` is the number of 3D points
                projected.
        """
        camera = self.cameras[index]
        pts, cams = self.ret_local_map(index)
        print (pts[:5])
        print ("Number of map points: {}".format(len(pts)))
        print (cams)
        pt = pts[0].project_point(index, camera=camera)
        print (pt)
        for point in pts:
            pt = np.vstack((pt, point.project_point(index, camera=camera)))
        pt = np.delete(pt, (0), axis=0)
        return pt
