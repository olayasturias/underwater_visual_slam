import numpy as np
import cv2
import pdb

"""
.. codeauthor::
: file Matcher.py

"""


class Matcher(object):
    """ The Matcher object allow us to find and store the keypoints of an image.

    We can use the ORB_ detector or the Flann_ detector. By default the object
    will use the ORB detector and the brute force matcher, but we can make
    use of the Flann-based matcher (with ORB or another detector) by
    passing to the object a dictionary with the detector and the matcher name.

    .. _ORB: http://www.willowgarage.com/sites/default/files/orb_final.pdf
    .. _Flann: http://docs.opencv.org/3.0-beta/doc/tutorials/features2d/feature_flann_matcher/feature_flann_matcher.html
    .. _KeyPoint: http://docs.opencv.org/3.0-beta/modules/core/doc/basic_structures.html#keypoint
    .. _Matcher: http://docs.opencv.org/3.0-beta/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html#descriptormatcher-knnmatch
    .. _Lowe: http://www.cs.ubc.ca/~lowe/papers/ijcv04.pdf#page=20
    .. _arrowedLine: http://docs.opencv.org/3.0-beta/modules/imgproc/doc/drawing_functions.html
    .. _Lucas-Kanade: https://cecas.clemson.edu/~stb/klt/lucas_bruce_d_1981_1.pdf
    .. _LK: http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_video/py_lucas_kanade/py_lucas_kanade.html#lucas-kanade-optical-flow-in-opencv


    For Flann based matcher we need to pass two dictionaries which specifies the
    algorithm to be used, its related parameters, etc. First one is
    *IndexParams*. We can use one of the following algorithms:

        1. Linear brute force search: FLANN_INDEX_LINEAR (0)
        2. KDTree search. The matcher will construct a set of randomized
           kd-trees which will be searched in parallel: FLANN_INDEX_KDTREE(1)
        3. KMeans search. The matcher will construct an hierarchical k-means
           tree: FLANN_INDEX_KMEANS (2)

    **Attributes**:

        .. data:: detector

           The detector instance

        .. data:: matcher

           The matcher instance

        .. data:: flann_algorithm

           Dictionary containing the different possible algorithms for the
           Flann based matcher.

        .. data:: norm

           The norm to be used if the brute force matcher is used. Can be
           *L2* norm, if not using *ORB* detector or *Hamming* otherwise.

        .. data:: kp1

           List of KeyPoints objects found by the detector in the first image.

        .. data:: kp2

           List of KeyPoints objects found by the detector in the second image.

        .. data:: desc1

           Numpy ndarray returned by the detector in the first image. This multi-
           dimensional array will have two dimensions:

               1. Array that stores the  descriptors:

                   .. code-block:: python

                      desc1[0] # Shows the first descriptor

                 We can get the number of descriptors returned by the detector:

                   .. code-block:: python

                      n_desc = (np.shape(desc1)[0])

               2. The descriptor itself. In the case of the *ORB* descriptor
                  it will be of size 32 (see the original paper of ORB_).

        .. data:: desc2

          Numpy ndarray returned by the detector in the second image.

          .. seealso::

              Attribute :py:mod:`desc1`

        .. data:: ratio

           This parameter is used in the calculation of the distance threshold.
           This threshold is used, in turn, to compute the distance measure for
           filtering the descriptors returned by the detectors.

        .. data:: matches1

           List of DMatch objects returned by the matcher when applied between
           the first image and the second image.

        .. data:: matches2

           List of DMatch objects returned by the matcher when applied between
           the second image and the first image.

        .. data:: good_matches

           List of DMatch objects filtered by an assymetric filter and a distance
           filter.

          .. seealso::

              Method :py:func:`Matcher.filter_distance`

              Method :py:func:`Matcher.filter_asymmetric`

        .. data:: good_kp1

           List of KeyPoint_ objects filtered (for the first image)

        .. data:: good_kp2

           List of KeyPoint_ objects filtered (for the second image)

        .. data:: good_desc1

           List of Numpy ndarrays corresponding to the filtered descriptors
           for the image 1 (the previous one)

        .. data:: good_desc2

           List of Numpy ndarrays corresponding to the filtered descriptors
           for the image 2 (the last one)


    **Constructor**:

        The constructor takes as argument a dictionary that contains the
        following keys:

            1. detector: Can be 'orb', 'surf' or 'sift'.

            2. matcher: Can be 'brute_force' or 'flann'.

            3. flann_index_params: The Flann based matcher takes different
            parameters depending on the used algorithm. This argument should
            be a dictionary containing the following keys:

                1. *LSH* algorithm:

                    a. algorithm: The algorithm to be used (*FLANN_LSH*)
                    b. table_number: The number of hash tables used (between
                       10 and 30 usually, but in the documentation they said
                       that 6 usually works better.
                    c. key_size: The size of the hash key in bits (between
                       10 and 20 usually)
                    d. multi_probe_level: The number of bits to shift to check
                       for neighboring buckets (0 is regular LSH, 2 is recommen-
                       ded).

                2. *KDTREE* algorithm:

                    a. algorithm: The algorithm to be used (*FLANN_KDTREE*)
                    b. trees: The number of parallel kd-trees to use. Good
                       values are in the range [1..16].

                :example:

                    >>> flann_index_params = dict(algorithm='FLANN_LSH',
                                                  table_number=6,
                                                  key_size=12,
                                                  multi_probe_level=1)
                    >>> flann_index_params = dict(algorithm='FLANN_KDTREE',
                                                  trees=5)

            4. flann_search_params: The Flann based matcher performs a K-nearest
               neighbor search for a given query point using the index. This
               search use a *checks* parameter that indicates the number of
               times the tree(s) in the index should be recursively traversed.
               A higher value for this parameter would give better search
               precision, but also take more time. This parameter is optional,
               so if we don't know which value should it takes then we can
               also pass an empty dictionary.

               :example:

                   >>> flann_search_params = dict(checks=50)




    """
    def __init__(self, parameters):
        # Initiate detector and matcher
        if parameters['detector'] == 'orb':
            self.feature = 'orb'
            self.detector = cv2.ORB_create(2000)
            self.norm = cv2.NORM_HAMMING
        elif parameters['detector'] == 'brisk':
            self.feature = 'brisk'
            self.detector = cv2.BRISK_create(30)
            self.norm = cv2.NORM_HAMMING
        elif parameters['detector'] == 'surf':
            self.feature = 'surf'
            self.detector = cv2.xfeatures2d.SURF_create(2500)
            self.norm = cv2.NORM_L2
        elif parameters['detector'] == 'sift':
            self.feature = 'sift'
            self.detector = cv2.xfeatures2d.SIFT_create(contrastThreshold = 0.07,
                                                        edgeThreshold = 5,
                                                        sigma = 2)
            self.norm = cv2.NORM_L2
        else:
            raise KeyError("Valid detectors: orb, surf, sift")
        if parameters['matcher'] == 'flann':
            self.matcher = cv2.FlannBasedMatcher(parameters['flann_index_params'],
                                                 parameters['flann_search_params'])
        else:
            # Use the brute force matcher. We set the *crossCheck* parameter to
            # False because we are going to check the matches ourselves.
            self.matcher = cv2.BFMatcher(self.norm, crossCheck=False)

        self.kp1 = []
        self.kp2 = []
        self.desc1 = None
        self.desc2 = None
        self.ratio = 0.65
        self.matches1 = None
        self.matches2 = None
        self.good_matches = None
        self.good_kp1 = []
        self.good_kp2 = []
        self.good_desc1 = []
        self.good_desc2 = []
        # The following three variables should'n be in this class
        # TODO: Remove these variables from this class
        self.global_matches = []
        self.global_kpts1 = []
        self.global_kpts2 = []
        self.lk_params = dict(winSize=(15, 15),
                              maxLevel=2,
                              criteria=(cv2.TERM_CRITERIA_EPS |
                                        cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    def match(self, img_prev, img_new):
        """ Match the two images

        This is the main method of the Matcher class. The algorithm is the
        following:

            1. Detect the KeyPoints and compute the corresponding descriptors
               for each image.

            2. Check if there are any KeyPoints (sometimes, specially if we
               use ROI's, it's possible that the detector return zero KeyPoints)

               If True, then check if the number of descriptors in both images
               are the same. If True, continue.

            3. Cross-Match the descriptors (from the first image to the second
               one and viceversa).

            4. Filter the matches (distance filter and assymetric filter).

        The filtered matches are stored in the :py:mod:`good_matches` attribute.

        :param img_new: The current (newest) image.
        :param img_prev: The previous image.
        :type img_new: Numpy ndarray
        :type img_prev: Numpy ndarray

        :returns:

                 * 0: No errors. There were enough matches.

                 * 1: Not enough KeyPoints in either the first or the second
                      image.

                 * 2: Not enough descriptors in either the first or the second
                      image.

                 * 3: No matches

        :rtype: Integer


        """
        self.good_kp1 = []
        self.good_kp2 = []
        self.good_desc1 = []
        self.good_desc2 = []
        self.good_matches = None
        self.kp1, self.desc1 = self.detector.detectAndCompute(img_prev, None)
        self.kp2, self.desc2 = self.detector.detectAndCompute(img_new, None)
        # There are any keypoints?
        if not self.kp1 or not self.kp2:
            return 1
        else:
            if np.size(self.desc1) == 0 or np.size(self.desc2) == 0:
                return 2
        # Cross compute matches
        self.matches1 = self.matcher.knnMatch(self.desc1, self.desc2, 2)
        self.matches2 = self.matcher.knnMatch(self.desc2, self.desc1, 2)

        # Check now if there are any matches
        if self.matches1 is None or self.matches2 is None:
            return 3
        else:
            self.good_matches = self.filter_matches(self.matches1,
                                                    self.matches2)
            return 0

    def filter_distance(self, matches):
        """ Filter the matches based on the distance between the best match
        and the second one.

        As D. Lowe_ stated, *for each feature point we have two candidate
        matches in the other
        view. These are the two best ones based on the distance between
        their descriptors. If this measured distance is very low for the
        best match, and much larger for the second best match, we can
        safely accept the first match as a good one since it is
        unambiguosly the best choice. Reciprocally, if the two best
        matches are relative close in distance, then there exists a
        possibility that we make an error if we select one or the other.
        In this case, we should reject both matches.*

        In his paper, D.Lowe recommend to use 0.65 as threshold distance, but
        this is valid only for the **SIFT**, so we are going to compute the
        distance threshold dynamically:

            1. First, get the distance for every match (including not only
               the best one, but also the second best one) and store it in a
               list.
            2. Then, compute the mean of the distances and multiply it with
               0.65. The result is the distance threshold.

        .. todo:: Maybe it would be better to use different thresholds based
                  on the detector. For example, if using SIFT then the threshold
                  would be 0.65. On the other hand, if using ORB_ (see Figure 5)
                  , then the threshold should be 64, and so on.

        :param matches: List of OpenCV Matcher_ objects
        :type matches: List of OpenCV Matcher_ objects
        :returns: List of Matcher objects filtered
        :rtype: List

        """
        dist = []
        sel_matches = []
        near_matches = []
        thres_dist = 0
        temp_matches = []

        if self.feature == 'orb':

            for i in range(0, len(matches) - 1):
                # We keep only those match objects with two matches:
                if (len(matches[i])) == 2:
                    # If there are two matches:
                    for j in range(0, 2):
                        dist.append(matches[i][j].distance)
                    temp_matches.append(matches[i])

            # Now, calculate the threshold:
            if dist:
                thres_dist = (sum(dist) / len(dist)) * self.ratio
            else:
                return None
            # Keep only reasonable matches based on the threshold distance:
            for i in range(0, len(temp_matches)):
                if (temp_matches[i][0].distance / temp_matches[i][1].distance) < thres_dist:
                    near_matches.append(temp_matches[i])

            # Sort them in the order of their distance.
            near_matches = sorted(near_matches, key = lambda x:np.minimum(x[1].distance,x[0].distance))
            sel_matches = near_matches[:50]

        else:
            # ratio test as per Lowe's paper
            for i,(m,n) in enumerate(matches):
                if m.distance < (self.ratio+0.1)*n.distance: #determined experimentally
                    sel_matches.append(matches[i])




        return sel_matches

    def filter_asymmetric(self, matches1, matches2):
        """ Cross-check the two matcher list

        We only want those matches that are bidirectional. To know if they are
        bidirectional we use the following algorithm:

            1. For each match in matches1 do:

                1.1 For each match in matches2 do:

                    1.1.1 If the index of the descriptor in the list of query
                          descriptors for matches1 (*desc1*) is the same
                          as the index of the descriptor in the train
                          descriptors for matches2 (which is also *desc1*)
                          **and** the index of the descriptor in the list of
                          query descriptors for matches2 (*desc2*) is the same
                          as the index of the descriptor in the list of train
                          descriptors for matches1 (*desc2*) then append the
                          match of matches1 to the final list.
                          Also, we store the KeyPoints in both images in two
                          attributes: *good_kp1* for the previous image, and
                          *good_kp2* for the current one.

                    1.1.2 Next

                1.2 Next

            2. End

        :param matches1: List of OpenCV Matcher_ objects (forwards)
        :param matches2: List of OpenCV Matcher_ objects (backwards)
        :type matches1: List of OpenCV Matcher_ objects
        :type matches2: List of OpenCV Matcher_ objects

        :returns: The filtered list of matches
        :rtype:  List of OpenCV Matcher_ objects



        """
        sel_matches = []
        # For every match in the forward direction, we remove those that aren't
        # found in the other direction
        for match1 in matches1:
            for match2 in matches2:
                # If matches are symmetrical:
                    if (match1[0].queryIdx) == (match2[0].trainIdx) and \
                         (match2[0].queryIdx) == (match1[0].trainIdx):
                        # We keep only symmetric matches and store the keypoints
                        # of this matches
                        sel_matches.append(match1)
                        self.good_kp2.append(self.kp2[match1[0].trainIdx])
                        self.good_kp1.append(self.kp1[match1[0].queryIdx])
                        self.good_desc1.append(self.desc1[match1[0].queryIdx])
                        self.good_desc2.append(self.desc2[match1[0].trainIdx])
                        break

        return sel_matches

    def filter_matches(self, matches1, matches2):
        """ This function filter two list of Matcher objects.

        The method calls two other methods:

            1. **filter_distance**, that filter the matches based on the distance
               between the best match and the second one (remember that we
               are using k-neighbors with 2 neighbors).

            2. **filter_assymetric**, that filter two list of matcher objects
               based on if they are bidirectional.

        After applying those filters the final KeyPoints and descriptors will
        be saved in the following attributes:

            * good_kp1
            * good_kp2
            * good_desc1
            * good_desc2

        :param matches1: List of matcher objects in the forward direction (i.e,
                         the query set is the previous image and the training
                         set is the current (last) image.
        :param matches2: List of matcher objects in the forward direction (i.e,
                         the query set is the current (last) image and the
                         training set is the previous image.

        :type matches1: List of OpenCV Matcher_ objects
        :type matches2: List of OpenCV Matcher_ objects
        :returns: If there were no errors, a list of OpenCV Matcher_ objects.
                  Else, None.
        :rtype: List

        """

        matches1 = self.filter_distance(matches1)
        matches2 = self.filter_distance(matches2)
        if matches1 and matches2:
            good_matches = self.filter_asymmetric(matches1, matches2)
            if good_matches:
                return good_matches
            else:
                return None
        else:
            return None

    def draw_matches(self, img, matches=None, kp1=None, kp2=None, color='red'):
        """ Draw the matches list in an image

        This method uses the OpenCV arrowedLine_ method to draw the matches in
        an image.

        We can pass it a list of Matcher objects (*matches* parameter) or
        two **Numpy ndarrays** (dimension nx2) representing the KeyPoints
        in the first and second image.

        The colors available are:

            1. Blue: (255, 0, 0)
            2. Green: (0, 255, 0)
            3. Red: (0, 0, 255)

        .. note::

            There is a difference in pixel ordering in OpenCV and Matplotlib.
            OpenCV follows BGR order, while matplotlib likely follows RGB order.
            Here, we are using the OpenCV color convention.

        :param img: The image where we are going to draw
        :param matches: The drawing list of matches
        :param kp1: KeyPoints in the first (previous) image
        :param kp2: KeyPoints in the second (last) image
        :type img: Numpy ndarray
        :type matches: List of OpenCV Matcher_ objects
        :type kp1: Numpy ndarray
        :type kp2: Numpy ndarray

        :returns: The modified image
        :rtype: Numpy ndarray

        """

        color_dict = dict(blue=(255, 0, 0),
                          green=(0, 255, 0),
                          red=(0, 0, 255))

        if matches is not None:
            for i in range(len(matches)):
                idtrain = matches[i][0].trainIdx
                idquery = matches[i][0].queryIdx

                point_train = self.kp2[idtrain].pt
                point_query = self.kp1[idquery].pt

                point_train = self.transform_float_int_tuple(point_train)
                point_query = self.transform_float_int_tuple(point_query)

                cv2.arrowedLine(img, ((point_query[0]), (point_query[1])),
                                ((point_train[0]), (point_train[1])),
                                color_dict[color],2)
        elif kp1 is not None and kp2 is not None:
            if np.shape(kp1) != np.shape(kp2):
                return img
            else:
                for i in range(len(kp1)):
                    cv2.arrowedLine(img, ((kp1[i][0]), (kp1[i][1])),
                                    ((kp2[i][0]), (kp2[i][1])),
                                    255)

        return img

    def transform_float_int_tuple(self, input_tuple):
        """ Converts a float tuple into an integers tuple

        :param input_tuple: The input tuple
        :type input_tuple: Tuple

        :returns: The tuple converted to integers
        :rtype: Tuple

        """
        output_tuple = [0, 0]
        if input_tuple is not None:
            for i in range(0, len(input_tuple)):
                output_tuple[i] = int(input_tuple[i])
        else:
            return input_tuple

        return output_tuple

    def kp_list_to_np(self, kp_list):
        """ Transforms a list of Keypoints into a numpy ndarray

        :param kp_list: List of KeyPoint_ objects
        :type kp_list: List

        :returns: A Numpy ndarray (nx2) with the KeyPoints coordinates
        :rtype: Numpy ndarray

        """
        temp = [item.pt for item in kp_list]
        return np.asarray(temp)

    def append_list(self, list1, list2):
        """ Appends a whole list to another one

        Appends list2 to list1.

        :param list1: The list to which we are going to append the other one
        :param list2: The list to be append
        :type list1: List
        :type list2: List

        :returns: A new list comprising the two previous ones.
        :rtype: List

        :raises: AssertionError
        """
        try:
            lst = list1.extend(list2)
            return lst
        except (AttributeError, TypeError):
            raise AssertionError('Input variables should be lists')

    def sum_coord(self, x_ini, y_ini):
        """ Sum x and y coordinates to the filtered KeyPoints

        If we are matching ROI's then the extracted KeyPoints will not have
        the correct coordinates (unless the ROI origin is the same as the
        image origin). So this function adds the initial coordinates of the ROI
        to the extracted KeyPoints (:py:mod:`good_kp1` and :py:mod:`good_kp2`).

        :param x_ini: ROI origin x coordinate
        :param y_ini: ROI origin y coordinate
        :type x_ini: Integer
        :type y_ini: Integer

        """
        for i in range(len(self.good_kp1)):
            # The pt attribute of the KeyPoint class is a tuple, so we can't
            # modify it. Therefore, we transform it to a list before.
            kp1 = list(self.good_kp1[i].pt)
            kp2 = list(self.good_kp2[i].pt)

            kp1[0] += x_ini
            kp1[1] += y_ini
            kp2[0] += x_ini
            kp2[1] += y_ini

            self.good_kp1[i].pt = (kp1[0], kp1[1])
            self.good_kp2[i].pt = (kp2[0], kp2[1])

    def lktracker(self, img_prev, img_curr, prev_points):
        """ Lucas-Kanade tracking algorithm

        This method applies the Lucas-Kanade_ (LK) algorithm to two images.
        OpenCV has a full and easy-use implementation of this algorithm (LK_),
        and here we just call it with the appropiate parameters.

        After computing the next KeyPoints we also compute the preious ones
        using the same algorithm but backwards (using the computed KeyPoints as
        the previous ones). Next, if the distance  between these last
        KeyPoints and the originals is greater than 2 pixels we reject the
        KeyPoints computed in the last (next) frame.

        :param img_prev: The previous frame
        :param img_curr: The next frame.
        :param prev_points: The previous KeyPoints
        :type img_prev: Numpy ndarray
        :type img_curr: Numpy ndarray
        :type prev_points: Numpy ndarray with shape nx2

        :returns: Three variables:

                    1. A numpy array (mask) that tell us which of the computed
                       KeyPoints are not rejected.

                    2. A numpy ndarray (nx2) with the computed KeyPoints in the
                       first (previous) image.

                    3. A numpy ndarray (mx2) with the tracked KeyPoints in the
                       next image.

        :rtype: Numpy ndarray

        .. note::

            The :py:mod:`calcOpticalFlowPyrLK` takes as input points a numpy
            ndarray with shape (n, 1, 2), and of type **numpy.float32**.

        """
        list_tracks = []
        prev_points_r = prev_points.reshape(prev_points.shape[0], 1, 2)
        prev_points_r = prev_points.astype(np.float32)
        curr_points, st, err = cv2.calcOpticalFlowPyrLK(img_prev,
                                                        img_curr,
                                                        prev_points_r,
                                                        None,
                                                        **self.lk_params)

        prev_points2, st, err = cv2.calcOpticalFlowPyrLK(img_curr,
                                                         img_prev,
                                                         curr_points,
                                                         None,
                                                         **self.lk_params)
        d = abs(prev_points - prev_points2).reshape(-1, 2).max(-1)
        good = d < 2
        for (x, y), good_flag in zip(curr_points.reshape(-1, 2), good):
            if not good_flag:
                continue
            list_tracks.append((x, y))

        return good, prev_points2, np.asarray(list_tracks)
