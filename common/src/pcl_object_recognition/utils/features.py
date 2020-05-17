import numpy as np
from pc_utils import scale_to_unit_sphere, center_and_rotate_pointcloud

class FVRDDFeatureExtraction():
    """
    PointCloud feature extraction. Available methods are fisher vector and \
    radial density distribution.

    :param method:      feature extraction method to use
    :type method:       string
    """
    def __init__(self, method="fvrdd"):
        if method == "fvrdd":
            self.method = self.calculate_fvrdd_features
        elif method == "rdd":
            self.method = self.calculate_mean_circle_features
    
    def get_method(self):
        """
        Get feature extraction method

        :return method:   a funtion to extract the feature
        :type use_rdd:    function
        """
        return self.method

    def set_fv_params(self, gmm_grid, use_rdd):
        """
        Set fisher vector (3DmFV) params

        :param gmm_grid:  The gmm grid (e.g. [2,2,2])
        :type gmm_grid:   numpy.array
        :param use_rdd:   whether to combine fv with rdd
        :type use_rdd:    bool
        """
        self.gmm = gmm_grid
        self.use_rdd = use_rdd
    
    def set_rdd_params(self, color=True):
        """
        Set radial density distribution feature param

        :param color:  The input pointcloud
        :type color:   numpy.array
        """
        self.use_color = color

    def calculate_centre_of_gravity_offset(self, pointcloud):
        """
        Returns difference between centre of gravity of the pointcloud
        in the x-axis and the origin

        :param pointcloud:  The input pointcloud
        :type pointcloud:   numpy.array

        :return:            The centre of gravity offset
        :rtype:             float

        """

        # The centre of gravity (CoG) of the pointcloud is assumed to be at 0,0,0 (after normalization)
        # The difference between the CoG and the geometric centre of the object ((max+min)/2) on the
        # x-axis gives the offset
        max_x = np.max(pointcloud[:, 0])
        min_x = np.min(pointcloud[:, 0])

        return (max_x + min_x) / 2.0


    def calculate_mean_colour(self, pointcloud):
        """
        Returns mean H, S and V components of colour

        :param pointcloud:  The input pointcloud
        :type pointcloud:   numpy.array

        :return:            mean H, S and V components of colour
        :rtype:             numpy.array

        """
        # fourth column of the array is colour represented as a 32bit float
        mean_h = np.mean(pointcloud[:, 3])
        mean_s = np.mean(pointcloud[:, 4])
        mean_v = np.mean(pointcloud[:, 5])

        return np.array([mean_h, mean_s, mean_v])


    def calculate_median_colour(self, pointcloud):
        """
        Returns median H, S and V components of colour

        :param pointcloud:  the input pointcloud
        :type pointcloud:   numpy.array

        :return:            median H, S and V components colour
        :rtype:             numpy.array

        """
        # fourth column of the array is colour represented as a 32bit float
        median_h = np.median(pointcloud[:, 3])
        median_s = np.median(pointcloud[:, 4])
        median_v = np.median(pointcloud[:, 5])

        return np.array([median_h, median_s, median_v])


    def calculate_bounding_box(self, pointcloud):
        """
        Returns length of pointcloud in x, y and z axes

        :param pointcloud:  the input pointcloud
        :type pointcloud:   numpy.array

        :return:            lengths of pointcloud in x, y and z axes
        :rtype:             numpy.array

        """
        pc = pointcloud[:, 0:3]
        max_point = np.max(pc, axis=0)
        min_point = np.min(pc, axis=0)

        return max_point - min_point


    def fit_circle(self, pointcloud, dim1, dim2):
        """
        Calculates mean circle, inlier and outlier error and radial density
        of pointcloud on the plane defined by the axes dim1 and dim2
        A visual representation of the radial density feature can be seen in common/doc/

        :param pointcloud:      the input pointcloud
        :type pointcloud:       numpy.array

        :param dim1:            index of first axis defining the plane
        :type dim1:             int

        :param dim2:            index of second axis defining the plane
        :type dim2:             int

        :return radius:         radius of mean circle
        :rtype radius:          float

        :return inlier_error:   inlier error (circle fit error of points lying inside the mean circle)
        :rtype inlier_error:    float

        :return outerr:         outlier error (circle fit error of points lying outside the mean circle)
        :rtype outerr:          float

        :return radial_density: mean normalized radial density for the pointcloud
        :rtype: radial_density: float

        """
        # Only get axes which are defined by dim1 and dim2
        pointcloud = np.vstack([pointcloud[:, dim1], pointcloud[:, dim2]]).T

        # Number of sections the circumference of the circle is split into
        # Determined experimentally with the objective of getting a "reasonable" number of
        # points per bin
        number_of_bins = 33

        histogram = np.zeros([number_of_bins])

        # Find the angle on a circle centred at the origin that the points are located at
        projected_angles = np.arctan2(pointcloud[:, 0], pointcloud[:, 1])

        # move range of angles from (-pi/2,pi/2) to (0, pi)
        negative_angles = np.where(projected_angles < 0)
        projected_angles[negative_angles] += 2 * np.pi

        # convert angle to bin number
        bin_numbers = np.round((number_of_bins-1) * projected_angles / (2 * np.pi))

        # populate the histogram
        for bin_number in bin_numbers:
            histogram[int(bin_number)] += 1

        # if there are points in any of the bins calculate radial density
        # otherwise return 0 for all features (radius, errors, radial density)
        if np.max(histogram) > 0:
            radial_density = np.sum(histogram / np.max(histogram)) / number_of_bins
        else:
            return 0, 0, 0, 0

        # radius of mean circle - mean distance of all points from the origin
        distance_to_origin = np.sqrt(np.sum(np.multiply(pointcloud[:, 0:2], pointcloud[:, 0:2]), axis=1))
        radius = np.mean(distance_to_origin)

        diff = distance_to_origin - radius
        inlier_errors = diff[np.where(diff < 0)]
        outlier_errors = diff[np.where(diff >= 0)]

        # mean squared error
        inlier_error = np.mean(np.multiply(inlier_errors, inlier_errors))
        outlier_error = np.mean(np.multiply(outlier_errors, outlier_errors))

        return radius, inlier_error, outlier_error, radial_density


    def calculate_slices_description(self, pointcloud, number_of_slices):
        """
        Calculates mean circle radius, inlier and outlier error and radial density
        for slices of the pointcloud taken along the x-axis (first principal axis)

        :param pointcloud:  the input pointcloud
        :type pointcloud:   numpy.array

        :param number_of_slices:    number of slices to be created along x-axis
        :type number_of_slices:     int

        :return:            array of features mentioned above for each slice
        :rtype:             numpy.array

        """
        pointcloud_xyz = pointcloud[:, 0:3]
        min_x = np.min(pointcloud_xyz[:, 0])
        max_x = np.max(pointcloud_xyz[:, 0])

        length_of_x_axis = max_x - min_x
        step = length_of_x_axis / number_of_slices

        # slices are bounded by left and right edge
        # start from minimum x (left edge)
        left_edge = min_x
        right_edge = left_edge + step

        slice_size = pointcloud_xyz.shape[0]
        slices = np.zeros([number_of_slices, slice_size, pointcloud_xyz.shape[1]])

        for k in range(number_of_slices):
            # define the points in the slice as all points between left and right edge
            # "multiply" is used to combine two conditions
            slice = pointcloud_xyz[np.where(
                np.multiply(pointcloud_xyz[:, 0] >= left_edge,
                            pointcloud_xyz[:, 0] < right_edge))]
            if slice.shape[0] < 1:
                continue
            slices[k, 0:slice.shape[0], :] = slice
            #print ("slice shape: {}, k: {}, slices shape: {}".format(slice.shape, k, slices.shape))
            # calculate features for the slice
            radius, inlier_error, outlier_error, radial_density = self.fit_circle(slice, 1, 2)
            slices[k, slice.shape[0], :] = np.array([radius, outlier_error/inlier_error, radial_density])

            # move to the next slice
            left_edge += step
            right_edge += step

        return slices

    def calculate_mean_circle_features(self, pointcloud, enable_color=False):
        """
        Calculates features for the input pointcloud
        Features are:
            bounding box
            centre of gravity offset
            mean colour
            median colour
            X-Y plane features
                mean circle
                ratio of outlier to inlier error
                radial density
            X axis slice features
                mean circle
                ratio of outlier to inlier error
                radial density

        A visual representation of some of these features can be seen in common/doc/

        :param pointcloud:      the input pointcloud
        :type pointcloud:       numpy.array

        :param enable_colour:   flag to specify if colour is specified and is to be used as a feature
        :type enable_colour:    boolean

        :return:                feature vector with the features mentioned above
        :rtype:                 numpy.array

        """
        pointcloud = center_and_rotate_pointcloud(pointcloud)

        # BBox feature: 3
        xyz = self.calculate_bounding_box(pointcloud)

        # CoG: 1
        centre_of_gravity_offset = self.calculate_centre_of_gravity_offset(pointcloud)

        features = np.array([])
        features = np.append(features, xyz)
        features = np.append(features, centre_of_gravity_offset)

        if self.use_color:
            colour_mean = self.calculate_mean_colour(pointcloud)
            colour_median = self.calculate_median_colour(pointcloud)
            features = np.append(features, colour_mean)
            features = np.append(features, colour_median)

        # Fit circle for whole points
        # plane radius: 1, plane radial density: 1, plane_error_rate: 1
        plane_radius, plane_inlier_error, plane_outlier_error, plane_radial_density = fit_circle(pointcloud, 0, 1)
        plane_error_rate = plane_outlier_error / plane_inlier_error
        features = np.append(features, plane_radius)
        features = np.append(features, plane_error_rate)
        features = np.append(features, plane_radial_density)

        slices = self.calculate_slices_description(pointcloud, 8)
        for k in range(slices.shape[0]):
            slice = slices[k, :, :]
            slice = slice[np.where(np.sum(np.abs(slice),axis=1) > 0)]
            if slice.shape[0] < 1:
                features = np.append(features, np.zeros([3]))
            else:
                features = np.append(features, slice[-1, :])

        return features

    def calculate_slice_features(self, pointcloud, number_of_slices):
        """
        Calculates slice features from different plane XY, XZ and YZ

        :param pointcloud:            the input pointcloud
        :type pointcloud:             numpy.array
        :param number_of_slices:      number of slices
        :type number_of_slices:       int

        :return:                      slice features
        :rtype:                       numpy.array

        """
        axis_features = []
        pointcloud_xyz = pointcloud[:, 0:3]
        #print (pointcloud.shape)
        for i in range(3):
            min_x = np.min(pointcloud_xyz[:, i])
            max_x = np.max(pointcloud_xyz[:, i])

            length_of_x_axis = max_x - min_x
            step = length_of_x_axis / number_of_slices

            # slices are bounded by left and right edge
            # start from minimum x (left edge)
            left_edge = min_x
            right_edge = left_edge + step

            slice_size = pointcloud_xyz.shape[0]
            
            slices = np.zeros([number_of_slices, slice_size, pointcloud_xyz.shape[1]])

            if i == 0:
                dim1 = 0
                dim2 = 1
            elif i == 1:
                dim1 = 0
                dim2 = 2
            elif i == 2:
                dim1 =1
                dim2 = 2

            for k in range(number_of_slices):
                # define the points in the slice as all points between left and right edge
                # "multiply" is used to combine two conditions
                slice = pointcloud_xyz[np.where(
                    np.multiply(pointcloud_xyz[:, i] >= left_edge,
                                pointcloud_xyz[:, i] < right_edge))]
                if slice.shape[0] < 1:
                    continue
                slices[k, 0:slice.shape[0], :] = slice
                # calculate features for the slice
                radius, inlier_error, outlier_error, radial_density = self.fit_circle(slice, dim1, dim2)
                slices[k, slice.shape[0], :] = np.array([radius, outlier_error/inlier_error, radial_density])
                # move to the next slice
                left_edge += step
                right_edge += step
                        
            axis_features.append(slices)
        
        return axis_features

    # Compute Fisher Vector
    def calculate_fvrdd_features(self, pointcloud):
        """
        Calculates fisher vector and multi axes radial density distribution features

        :param pointcloud:      the input pointcloud
        :type pointcloud:       numpy.array

        :return:                combined feature vector
        :rtype:                 numpy.array

        """
        pointcloud = center_and_rotate_pointcloud(pointcloud)

        features = []
        slice_features = self.calculate_slice_features(pointcloud, 8)
        slice_features = np.asarray(slice_features)
        for slices in slice_features:
            s_feature = []
            for k in range(slices.shape[0]):
                slice = slices[k, :, :]
                slice = slice[np.where(np.sum(np.abs(slice),axis=1) > 0)]
                if slice.shape[0] < 1:
                    s_feature.append(np.zeros([3]))
                else:
                    s_feature.append(slice[-1, :])
            s_feature = np.asarray(s_feature).flatten()  
            features.extend(s_feature)
        
        # Also compute additional radial density distribution feature feature 
        # such as color, BBox, outlier error
        if self.use_rdd:
            plane_radius, plane_inlier_error, plane_outlier_error, plane_radial_density = self.fit_circle(pointcloud, 0, 1)
            plane_error_rate = np.divide(plane_outlier_error, plane_inlier_error)
            features.append(plane_radius)
            features.append(plane_error_rate)
            features.append(plane_radial_density)

            bbox = self.calculate_bounding_box(pointcloud)
            centre_of_gravity_offset = self.calculate_centre_of_gravity_offset(pointcloud)
            
            # Bounding box does improve classifier
            features.append(bbox[0])
            features.append(bbox[1])
            features.append(bbox[2])
            features.append(centre_of_gravity_offset)

            # Add color feature
            colour_mean = self.calculate_mean_colour(pointcloud)
            colour_median = self.calculate_median_colour(pointcloud)
            features.append(colour_mean[0])
            features.append(colour_mean[1])
            features.append(colour_mean[2])
            features.append(colour_median[0])
            features.append(colour_median[1])
            features.append(colour_median[2])

        rdd_features = np.asarray(features).flatten()
        
        # Use color feature
        pcl_color = pointcloud[:,3:6]
        pcl_color = center_and_rotate_pointcloud(pcl_color)
        pcl_color = scale_to_unit_sphere(pcl_color)
        pcl_color = np.expand_dims(pcl_color, 0)
        fv_color = self.get_3DmFV(pcl_color, self.gmm.weights_, self.gmm.means_, np.sqrt(self.gmm.covariances_))
        
        # Point feature
        pointcloud = pointcloud[:,0:3]
        pointcloud = center_and_rotate_pointcloud(pointcloud)
        pointcloud = scale_to_unit_sphere(pointcloud)
        pointcloud = np.expand_dims(pointcloud, 0)
        fv = self.get_3DmFV(pointcloud, self.gmm.weights_, self.gmm.means_, np.sqrt(self.gmm.covariances_))
        fv = np.concatenate([fv_color, fv])
        fv = fv.flatten()

        rvrdd_features = np.hstack((rdd_features, fv))
        
        return rvrdd_features


    def l2_normalize(self, v, dim=1):
        """
        Normalize a vector along a dimension

        :param v: a vector or matrix to normalize
        :param dim: the dimension along which to normalize
        :return: normalized v along dim
        """
        norm = np.linalg.norm(v, axis=dim)
        if norm.all() == 0:
            return v
        return v / norm

    def get_3DmFV(self, points, w, mu, sigma, normalize=True):
        """
        Compute the 3D modified fisher vectors given the gmm model parameters (w,mu,sigma) and a set of points

        :param points: B X N x 3 tensor of XYZ points
        :param w: B X n_gaussians tensor of gaussian weights
        :param mu: B X n_gaussians X 3 tensor of gaussian cetnters
        :param sigma: B X n_gaussians X 3 tensor of stddev of diagonal covariance
        :return: fv: B X 20*n_gaussians tensor of the fisher vector
        """
        n_batches = points.shape[0]
        n_points = points.shape[1]
        n_gaussians = mu.shape[0]
        D = mu.shape[1]

        # Expand dimension for batch compatibility
        batch_sig = np.tile(np.expand_dims(sigma, 0), [n_points, 1, 1])  
        batch_sig = np.tile(np.expand_dims(batch_sig, 0), [n_batches, 1, 1, 1]) 
        batch_mu = np.tile(np.expand_dims(mu, 0), [n_points, 1, 1]) 
        batch_mu = np.tile(np.expand_dims(batch_mu, 0), [n_batches, 1, 1, 1])  
        batch_w = np.tile(np.expand_dims(np.expand_dims(w, 0), 0), [n_batches, n_points, 1])  
        batch_points = np.tile(np.expand_dims(points, -2), [1, 1, n_gaussians, 1]) 
        # Compute derivatives
        w_per_batch_per_d = np.tile(np.expand_dims(np.expand_dims(w, 0), -1),
                                    [n_batches, 1, 3*D])  
        # Define multivariate noraml distributions
        # Compute probability per point
        p_per_point = (1.0 / (np.power(2.0 * np.pi, D / 2.0) * np.power(batch_sig[:, :, :, 0], D))) * np.exp(
            -0.5 * np.sum(np.square((batch_points - batch_mu) / batch_sig), axis=3))

        w_p = p_per_point
        Q = w_p  # enforcing the assumption that the sum is 1
        Q_per_d = np.tile(np.expand_dims(Q, -1), [1, 1, 1, D])

        d_pi_all = np.expand_dims((Q - batch_w) / (np.sqrt(batch_w)), -1)
        d_pi = np.concatenate([np.max(d_pi_all, axis=1), np.sum(d_pi_all, axis=1)], axis=2)

        d_mu_all = Q_per_d * (batch_points - batch_mu) / batch_sig
        d_mu = (1 / (np.sqrt(w_per_batch_per_d))) * np.concatenate([np.max(d_mu_all, axis=1), 
                np.min(d_mu_all, axis=1), np.sum(d_mu_all, axis=1)], axis=2)

        d_sig_all = Q_per_d * (np.square((batch_points - batch_mu) / batch_sig) - 1)
        d_sigma = (1 / (np.sqrt(2 * w_per_batch_per_d))) * np.concatenate([np.max(d_sig_all, axis=1), 
                    np.min(d_sig_all, axis=1), np.sum(d_sig_all, axis=1)], axis=2)

        # number of points  normaliation
        d_pi = d_pi / n_points
        d_mu = d_mu / n_points
        d_sigma =d_sigma / n_points

        if normalize:
            # Power normaliation
            alpha = 0.5
            d_pi = np.sign(d_pi) * np.power(np.abs(d_pi), alpha)
            d_mu = np.sign(d_mu) * np.power(np.abs(d_mu), alpha)
            d_sigma = np.sign(d_sigma) * np.power(np.abs(d_sigma), alpha)

            # L2 normaliation
            d_pi = np.array([self.l2_normalize(d_pi[i, :, :], dim=0) for i in range(n_batches)])
            d_mu = np.array([self.l2_normalize(d_mu[i, :, :], dim=0) for i in range(n_batches)])
            d_sigma = np.array([self.l2_normalize(d_sigma[i, :, :], dim=0) for i in range(n_batches)])


        fv = np.concatenate([d_pi, d_mu, d_sigma], axis=2)
        fv = np.transpose(fv, axes=[0, 2, 1])

        return fv