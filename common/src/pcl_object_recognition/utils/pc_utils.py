import struct
import numpy as np
from sklearn.mixture import GaussianMixture
from sklearn.preprocessing import normalize
from sklearn.decomposition import PCA

# copied from python-pcl
def float_to_rgb(p_rgb):
    # rgb = *reinterpret_cast<int*>(&p.rgb)
    rgb_bytes = struct.pack('f', p_rgb)
    rgb = struct.unpack('I', rgb_bytes)[0]
    r = (rgb >> 16) & 0x0000ff
    g = (rgb >> 8)  & 0x0000ff
    b = (rgb)       & 0x0000ff
    return (r/255.0),(g/255.0),(b/255.0)

def scale_to_unit_sphere(points):
    """
    Scale point cloud to unit sphere
    """
    centroid = np.mean(points, axis=0)
    points = points - centroid
    scale = np.max(np.sqrt(np.sum(points**2, axis=1)))
    if scale > 0.0:
        points = points / scale

    return points  

def pca_compress(pointcloud, n_components=3):
    """
    Find principal axes of the pointcloud

    :param pointcloud:  The input pointcloud
    :type pointcloud:   numpy.array

    :return:            PCA object fit to the given pointcloud
    :rtype:             sklearn.decomposition.PCA

    """
    pca = PCA(n_components)
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

def get_3d_grid_gmm(subdivisions=[5,5,5], variance=0.04):
    """
    Compute the weight, mean and covariance of a gmm placed on a 3D grid
    :param subdivisions: 2 element list of number of subdivisions of the 3D space in each axes to form the grid
    :param variance: scalar for spherical gmm.p
    :return gmm: gmm: instance of sklearn GaussianMixture (GMM) object Gauassian mixture model
    """
    # n_gaussians = reduce(lambda x, y: x*y,subdivisions)
    n_gaussians = np.prod(np.array(subdivisions))
    step = [1.0/(subdivisions[0]),  1.0/(subdivisions[1]),  1.0/(subdivisions[2])]

    means = np.mgrid[ step[0]-1: 1.0-step[0]: complex(0, subdivisions[0]),
                      step[1]-1: 1.0-step[1]: complex(0, subdivisions[1]),
                      step[2]-1: 1.0-step[2]: complex(0, subdivisions[2])]
    means = np.reshape(means, [3, -1]).T
    covariances = variance*np.ones_like(means)
    weights = (1.0/n_gaussians)*np.ones(n_gaussians)
    gmm = GaussianMixture(n_components=n_gaussians, covariance_type='diag')
    gmm.weights_ = weights
    gmm.covariances_ = covariances
    gmm.means_ = means
    from sklearn.mixture.gaussian_mixture import _compute_precision_cholesky
    gmm.precisions_cholesky_ = _compute_precision_cholesky(covariances, 'diag')
    return gmm