#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>

extern "C"
{
  /** Estimate rigid transform between two sets of points.
    *
    * @param a : array of floats, where first triple of numbers correspond to
    * the coordinates of the first point in the first cloud, and so on.
    *
    * @param b : array of coordinates of the points in the second cloud.
    *
    * @param point_count : number of points in clouds, should be equal to the
    * number of elements in @arg a or @arg b divided by 3.
    *
    * @param transform : pointer to an array of 6 floats, which will be filled
    * with x, y, z, roll, pitch, and yaw of the estimated transform. */
  void estimateTransformation(float* a, float* b, int point_count, float* transform)
  {
    pcl::PointCloud<pcl::PointXYZ> A, B;
    for (int i = 0; i < point_count; i++)
    {
      pcl::PointXYZ pa {a[i * 3], a[i * 3 + 1], a[i * 3 + 2]};
      pcl::PointXYZ pb {b[i * 3], b[i * 3 + 1], b[i * 3 + 2]};
      A.push_back(pa);
      B.push_back(pb);
    }
    Eigen::Matrix4f t;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> te;
    te.estimateRigidTransformation(A, B, t);
    Eigen::Transform<float, 3, Eigen::Affine> ta(t);
    Eigen::Vector3f rpy = ta.rotation().eulerAngles(0, 1, 2);
    Eigen::Vector3f xyz = ta.translation();
    transform[0] = xyz(0);
    transform[1] = xyz(1);
    transform[2] = xyz(2);
    transform[3] = rpy(0);
    transform[4] = rpy(1);
    transform[5] = rpy(2);
  }
}

