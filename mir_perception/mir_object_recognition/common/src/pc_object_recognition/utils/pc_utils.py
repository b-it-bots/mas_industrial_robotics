import struct
import numpy as np
from sklearn.mixture import GaussianMixture
from sklearn.mixture._gaussian_mixture import _compute_precision_cholesky
from sklearn.preprocessing import normalize
from sklearn.decomposition import PCA
# import open3d
# import open3d

# copied from python-pcl


def float_to_rgb(p_rgb):
    """
    Get rgb color from float rgb

    :param p_rgb:         24 bit packed rgb 
    :type:                numpy.array

    :return:            The rgb color
    :rtype:             numpy.array
    """
    # rgb = *reinterpret_cast<int*>(&p.rgb)
    rgb_bytes = struct.pack('f', p_rgb)
    rgb = struct.unpack('I', rgb_bytes)[0]
    r = (rgb >> 16) & 0x0000ff
    g = (rgb >> 8) & 0x0000ff
    b = (rgb) & 0x0000ff

    return (r/255.0), (g/255.0), (b/255.0)


def scale_to_unit_sphere(pointcloud, normalize=True):
    """
    Scale point cloud to unit sphere

    :param pointcloud:    The input pointcloud
    :type:                numpy.array
    :param normalize:     True if pointcloud needs to be normalized
    :type:                bool

    :return:            The rotated pointcloud
    :rtype:             numpy.array
    """
    if normalize:
        centroid = np.mean(pointcloud, axis=0)
        pointcloud = pointcloud - centroid

    scale = np.max(np.sqrt(np.sum(pointcloud**2, axis=1)))
    if scale > 0.0:
        pointcloud = pointcloud / scale

    return pointcloud


def pca_compress(pointcloud, n_components=3):
    """
    Find principal axes of the pointcloud

    :param pointcloud:    The input pointcloud
    :type pointcloud:     numpy.array

    :return:            PCA object fit to the given pointcloud
    :rtype:             sklearn.decomposition.PCA

    """
    pca = PCA(n_components)
    pca.fit(pointcloud)

    return pca


def rotate_pointcloud(normalized_pointcloud):
    """
    Find first three principal components and rotate pointcloud so the first principal
    component is aligned along the x-axis, 2nd principal component along y-axis and the
    third along the z-axis

    :param pointcloud:    The input pointcloud
    :type pointcloud:     numpy.array

    :return:            The rotated pointcloud
    :rtype:             numpy.array

    """
    pointcloud_xyz = normalized_pointcloud[:, 0:3]
    principal_components = pca_compress(pointcloud_xyz).components_
    squared_length_principal_components = np.multiply(
        principal_components, principal_components)
    length_principal_components = np.sqrt(
        np.sum(squared_length_principal_components, axis=1))

    # Calculate rotation matrix
    R = principal_components
    R[0, :] = R[0, :] / length_principal_components[0]
    R[1, :] = R[1, :] / length_principal_components[1]
    R[2, :] = R[2, :] / length_principal_components[2]

    pointcloud_xyz = R.dot(pointcloud_xyz.T).T

    # if colour is part of the pointcloud
    if normalized_pointcloud.shape[1] > 3:
        return np.hstack([pointcloud_xyz, normalized_pointcloud[:, 3:]])
    else:
        return pointcloud_xyz


def center_pointcloud(pointcloud):
    """
    Center pointcloud
    :param pointcloud:    The input pointcloud
    :type pointcloud:     numpy.array

    :return:            The rotated pointcloud
    :rtype:             numpy.array
    """
    pointcloud_xyz = pointcloud[:, 0:3]
    number_of_points = pointcloud_xyz.shape[0]
    centre = np.sum(pointcloud, axis=0) / number_of_points
    pointcloud_xyz[:, 0] -= centre[0]
    pointcloud_xyz[:, 1] -= centre[1]
    pointcloud_xyz[:, 2] -= centre[2]

    # if colour is part of the pointcloud
    if pointcloud.shape[1] > 3:
        return np.hstack([pointcloud_xyz, pointcloud[:, 3:]])
    else:
        return pointcloud_xyz


def center_and_rotate_pointcloud(pointcloud):
    """
    Transforms pointcloud so it is centered at (0,0,0) and
    its first three principal components are aligned along the x,y and z axes

    :param pointcloud:    The input pointcloud
    :type pointcloud:     numpy.array

    :return:            The centered and rotated pointcloud
    :rtype:             numpy.array

    """
    pointcloud = center_pointcloud(pointcloud)
    rotated_cloud = rotate_pointcloud(pointcloud[:, 0:3])

    if pointcloud.shape[1] > 3:
        return np.hstack([rotated_cloud, pointcloud[:, 3:]])
    else:
        return rotated_cloud


def get_3d_grid_gmm(subdivisions=[5, 5, 5], variance=0.04):
    """
    Compute the weight, mean and covariance of a gmm placed on a 3D grid
    :param subdivisions: 2 element list of number of subdivisions of the 3D space in each axes to form the grid
    :param variance: scalar for spherical gmm.p
    :return gmm: gmm: instance of sklearn GaussianMixture (GMM) object Gauassian mixture model
    """
    # n_gaussians = reduce(lambda x, y: x*y,subdivisions)
    n_gaussians = np.prod(np.array(subdivisions))
    step = [1.0/(subdivisions[0]),    1.0 /
            (subdivisions[1]),    1.0/(subdivisions[2])]

    means = np.mgrid[step[0]-1: 1.0-step[0]: complex(0, subdivisions[0]),
                     step[1]-1: 1.0-step[1]: complex(0, subdivisions[1]),
                     step[2]-1: 1.0-step[2]: complex(0, subdivisions[2])]
    means = np.reshape(means, [3, -1]).T
    covariances = variance*np.ones_like(means)
    weights = (1.0/n_gaussians)*np.ones(n_gaussians)
    gmm = GaussianMixture(n_components=n_gaussians, covariance_type='diag')
    gmm.weights_ = weights
    gmm.covariances_ = covariances
    gmm.means_ = means
    gmm.precisions_cholesky_ = _compute_precision_cholesky(covariances, 'diag')
    return gmm


def extract_pcd(pointcloud,
                num_points=2048,
                color=True,
                downsample_cloud=True,
                pad_cloud=True,
                normalize_cloud=True
                ):

    xyzrgb = pointcloud

    # pad cloud until its size == num_points
    if pad_cloud:
        while xyzrgb.shape[0] < num_points:
            rand_idx = np.random.randint(xyzrgb.shape[0])
            xyzrgb = np.vstack([xyzrgb, xyzrgb[rand_idx]])

    return xyzrgb
