#######!/usr/bin/env python

import numpy as np
import sklearn.decomposition
import utils


def pca_compress(pointcloud, n_components=3):
    """
    Find principal axes of the pointcloud

    :param pointcloud:  The input pointcloud
    :type pointcloud:   numpy.array

    :return:            PCA object fit to the given pointcloud
    :rtype:             sklearn.decomposition.PCA

    """
    pca = sklearn.decomposition.PCA(n_components)
    pca.fit(pointcloud)

    return pca


def normalize_pointcloud(pointcloud):
    """
    Transforms pointcloud so it is centered at (0,0,0) and
    its first three principal components are aligned along the x,y and z axes

    :param pointcloud:  The input pointcloud
    :type pointcloud:   numpy.array

    :return:            The normalized pointcloud
    :rtype:             numpy.array

    """
    # Subtract mean of points from all the points to centre the pointcloud at 0,0,0
    pointcloud_xyz = pointcloud[:, 0:3]
    number_of_points = pointcloud_xyz.shape[0]
    centre = np.sum(pointcloud, axis=0) / number_of_points
    pointcloud_xyz[:, 0] -= centre[0]
    pointcloud_xyz[:, 1] -= centre[1]
    pointcloud_xyz[:, 2] -= centre[2]

    # find first three principal components and rotate pointcloud so the first principal
    # component is aligned along the x-axis, 2nd principal component along y-axis and the
    # third along the z-axis
    principal_components = pca_compress(pointcloud_xyz).components_
    squared_length_principal_components = np.multiply(principal_components, principal_components)
    length_principal_components = np.sqrt(np.sum(squared_length_principal_components, axis=1))

    # Calculate rotation matrix
    R = principal_components
    R[0, :] = R[0, :] / length_principal_components[0]
    R[1, :] = R[1, :] / length_principal_components[1]
    R[2, :] = R[2, :] / length_principal_components[2]

    # rotate the pointcloud
    if pointcloud.shape[1] > 3:  # if colour is part of the pointcloud
        normalized_pointcloud = np.hstack([R.dot(pointcloud_xyz.T).T, pointcloud[:, 3:]])
    else:
        normalized_pointcloud = R.dot(pointcloud_xyz.T).T

    return normalized_pointcloud

def calculate_centre_of_gravity_offset(pointcloud):
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


def calculate_mean_colour(pointcloud):
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


def calculate_median_colour(pointcloud):
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


def calculate_bounding_box(pointcloud):
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


def fit_circle(pointcloud, dim1, dim2):
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


def calculate_slices_description(pointcloud, number_of_slices):
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
        radius, inlier_error, outlier_error, radial_density = fit_circle(slice, 1, 2)
        slices[k, slice.shape[0], :] = np.array([radius, outlier_error/inlier_error, radial_density])

        # move to the next slice
        left_edge += step
        right_edge += step

    return slices

def calculate_feature_vector(pointcloud, enable_color=False):
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
    pointcloud = normalize_pointcloud(pointcloud)

    # BBox feature: 3
    xyz = calculate_bounding_box(pointcloud)

    # CoG: 1
    centre_of_gravity_offset = calculate_centre_of_gravity_offset(pointcloud)

    features = np.array([])
    features = np.append(features, xyz)
    features = np.append(features, centre_of_gravity_offset)

    if enable_color:
        colour_mean = calculate_mean_colour(pointcloud)
        colour_median = calculate_median_colour(pointcloud)
        features = np.append(features, colour_mean)
        features = np.append(features, colour_median)

    # Fit circle for whole points
    # plane radius: 1, plane radial density: 1, plane_error_rate: 1
    plane_radius, plane_inlier_error, plane_outlier_error, plane_radial_density = fit_circle(pointcloud, 0, 1)
    plane_error_rate = plane_outlier_error / plane_inlier_error
    features = np.append(features, plane_radius)
    features = np.append(features, plane_error_rate)
    features = np.append(features, plane_radial_density)

    slices = calculate_slices_description(pointcloud, 8)
    for k in range(slices.shape[0]):
        slice = slices[k, :, :]
        slice = slice[np.where(np.sum(np.abs(slice),axis=1) > 0)]
        if slice.shape[0] < 1:
            features = np.append(features, np.zeros([3]))
        else:
            features = np.append(features, slice[-1, :])

    return features

def calculate_slice_features(pointcloud, number_of_slices):
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
            radius, inlier_error, outlier_error, radial_density = fit_circle(slice, dim1, dim2)
            slices[k, slice.shape[0], :] = np.array([radius, outlier_error/inlier_error, radial_density])
            # move to the next slice
            left_edge += step
            right_edge += step
                    
        axis_features.append(slices)
    
    return axis_features

# Compute Multi-Axes Radial Density Distribution and Fisher Vector
# Combine the feature
def calculate_maxrdd_fv_features(pointcloud, gmm, feature_extraction, mean_circle=False):
    pointcloud = normalize_pointcloud(pointcloud)

    features = []
    slice_features = calculate_slice_features(pointcloud, 8)
    slice_features = np.asarray(slice_features)
    #print (slice_features.shape)
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
    
    # Use mean circle feature such as color, BBox, outlier error
    if mean_circle:
        plane_radius, plane_inlier_error, plane_outlier_error, plane_radial_density = fit_circle(pointcloud, 0, 1)
        plane_error_rate = np.divide(plane_outlier_error, plane_inlier_error)
        features.append(plane_radius)
        features.append(plane_error_rate)
        features.append(plane_radial_density)

        bbox = calculate_bounding_box(pointcloud)
        centre_of_gravity_offset = calculate_centre_of_gravity_offset(pointcloud)
        
        # Bounding box does improve classifier
        features.append(bbox[0])
        features.append(bbox[1])
        features.append(bbox[2])
        features.append(centre_of_gravity_offset)

        # Add color feature
        colour_mean = calculate_mean_colour(pointcloud)
        colour_median = calculate_median_colour(pointcloud)
        features.append(colour_mean[0])
        features.append(colour_mean[1])
        features.append(colour_mean[2])
        features.append(colour_median[0])
        features.append(colour_median[1])
        features.append(colour_median[2])

    features = np.asarray(features)
    maxrdd = features.flatten()
    
    # Use color feature
    pcl_color = pointcloud[:,3:6]
    pcl_color = normalize_pointcloud(pcl_color)
    pcl_color = utils.scale_to_unit_sphere(pcl_color)
    pcl_color = np.expand_dims(pcl_color, 0)
    fv_color = utils.get_3DmFV(pcl_color, gmm.weights_, gmm.means_, np.sqrt(gmm.covariances_))
    
    # Point feature
    pointcloud = pointcloud[:,0:3]
    pointcloud = normalize_pointcloud(pointcloud)
    pointcloud = utils.scale_to_unit_sphere(pointcloud)
    pointcloud = np.expand_dims(pointcloud, 0)
    fv = utils.get_3DmFV(pointcloud, gmm.weights_, gmm.means_, np.sqrt(gmm.covariances_))
    fv = np.concatenate([fv_color, fv])
    fv = fv.flatten()

    if feature_extraction == 'msc_fv_1' or feature_extraction == 'msc_fv_2' or \
        feature_extraction == 'msc_fv_3' or feature_extraction == 'msc_fv_4' or \
        feature_extraction == 'msc_fv_5':
        maxrdd_fv_features = np.hstack((maxrdd, fv))
        maxrdd_fv_features = np.asarray(maxrdd_fv_features)

    elif feature_extraction == 'fv_1' or feature_extraction == 'fv_2' or \
        feature_extraction == 'fv_3' or feature_extraction == 'fv_4' or \
        feature_extraction == 'fv_5':
        maxrdd_fv_features = fv
    #print (maxrdd_fv_features)
    return maxrdd_fv_features
