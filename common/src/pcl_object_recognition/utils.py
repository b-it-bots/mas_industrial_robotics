# Modified 3DmFV with cloud alignment and downsampling using Open3D
# by Mohammad Wasil https://github.com/mhwasil/cnn_pointcloud_classification
# Original 3DmFV source by  Ben-Shabat et al. https://github.com/sitzikbs/3DmFV-Net

from sklearn.mixture import GaussianMixture
from sklearn.preprocessing import normalize
import numpy as np
from sklearn.decomposition import PCA

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

def l2_normalize(v, dim=1):
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


def get_3DmFV(points, w, mu, sigma, normalize=True):
    """
       Compute the 3D modified fisher vectors given the gmm model parameters (w,mu,sigma) and a set of points
       For faster performance (large batches) use the tensorflow version

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
    batch_sig = np.tile(np.expand_dims(sigma, 0), [n_points, 1, 1])  # n_points X n_gaussians X D
    batch_sig = np.tile(np.expand_dims(batch_sig, 0), [n_batches, 1, 1, 1])  # n_batches X n_points X n_gaussians X D
    batch_mu = np.tile(np.expand_dims(mu, 0), [n_points, 1, 1])  # n_points X n_gaussians X D
    batch_mu = np.tile(np.expand_dims(batch_mu, 0), [n_batches, 1, 1, 1])  # n_batches X n_points X n_gaussians X D
    batch_w = np.tile(np.expand_dims(np.expand_dims(w, 0), 0), [n_batches, n_points,
                                                                1])  # n_batches X n_points X n_guassians X D  - should check what happens when weights change
    batch_points = np.tile(np.expand_dims(points, -2), [1, 1, n_gaussians,
                                                        1])  # n_batchesXn_pointsXn_gaussians_D  # Generating the number of points for each gaussian for separate computation

    # Compute derivatives
    w_per_batch_per_d = np.tile(np.expand_dims(np.expand_dims(w, 0), -1),
                                [n_batches, 1, 3*D])  # n_batches X n_gaussians X 3*D (D for min and D for max)

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
    d_mu = (1 / (np.sqrt(w_per_batch_per_d))) * np.concatenate([np.max(d_mu_all, axis=1), np.min(d_mu_all, axis=1), np.sum(d_mu_all, axis=1)], axis=2)

    d_sig_all = Q_per_d * (np.square((batch_points - batch_mu) / batch_sig) - 1)
    d_sigma = (1 / (np.sqrt(2 * w_per_batch_per_d))) * np.concatenate([np.max(d_sig_all, axis=1), np.min(d_sig_all, axis=1), np.sum(d_sig_all, axis=1)], axis=2)

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
        d_pi = np.array([l2_normalize(d_pi[i, :, :], dim=0) for i in range(n_batches)])
        d_mu = np.array([l2_normalize(d_mu[i, :, :], dim=0) for i in range(n_batches)])
        d_sigma = np.array([l2_normalize(d_sigma[i, :, :], dim=0) for i in range(n_batches)])


    fv = np.concatenate([d_pi, d_mu, d_sigma], axis=2)
    fv = np.transpose(fv, axes=[0, 2, 1])

    return fv

def scale_to_unit_sphere(points):
    centroid = np.mean(points, axis=0)
    points = points - centroid
    scale = np.max(np.sqrt(np.sum(points**2, axis=1)))
    if scale > 0.0:
        points = points / scale

    return points  
