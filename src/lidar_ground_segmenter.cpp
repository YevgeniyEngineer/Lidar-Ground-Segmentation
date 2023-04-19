#include "lidar_ground_segmenter.hpp"

namespace segmentation
{
IndexedPoint3D::IndexedPoint3D(double x, double y, double z, std::size_t index) : x{x}, y{y}, z{z}, index{index}
{
}

Plane3D::Plane3D(double nx, double ny, double nz, double d) : normal{nx, ny, nz}, d{d}
{
}

LidarGroundSegmenter::LidarGroundSegmenter(const std::vector<Point3D> &points, std::uint8_t number_of_iterations,
                                           std::uint8_t number_of_planar_partitions,
                                           std::uint16_t number_of_lowest_point_representatives,
                                           double sensor_height_offset_m, double distance_threshold_m,
                                           double initial_seed_threshold_m)
    : number_of_iterations_{number_of_iterations}, number_of_planar_partitions_{number_of_planar_partitions},
      number_of_lowest_point_representatives_{number_of_lowest_point_representatives},
      sensor_height_offset_m_{sensor_height_offset_m}, distance_threshold_m_{distance_threshold_m},
      initial_seed_threshold_m_{initial_seed_threshold_m}
{
    processing_points_.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        const auto &point = points[i];
        processing_points_.push_back(IndexedPoint3D{point[0], point[1], point[2], i});
    }
}

const std::uint8_t &LidarGroundSegmenter::number_of_iterations() const noexcept
{
    return number_of_iterations_;
}
std::uint8_t &LidarGroundSegmenter::number_of_iterations() noexcept
{
    return number_of_iterations_;
}
void LidarGroundSegmenter::number_of_iterations(std::uint8_t number_of_iterations)
{
    number_of_iterations_ = number_of_iterations;
}

const std::uint8_t &LidarGroundSegmenter::number_of_planar_partitions() const noexcept
{
    return number_of_planar_partitions_;
}
std::uint8_t &LidarGroundSegmenter::number_of_planar_partitions() noexcept
{
    return number_of_planar_partitions_;
}
void LidarGroundSegmenter::number_of_planar_partitions(std::uint8_t number_of_planar_partitions)
{
    number_of_planar_partitions_ = number_of_planar_partitions;
}

const std::uint16_t &LidarGroundSegmenter::number_of_lowest_point_representatives() const noexcept
{
    return number_of_lowest_point_representatives_;
}
std::uint16_t &LidarGroundSegmenter::number_of_lowest_point_representatives() noexcept
{
    return number_of_lowest_point_representatives_;
}
void LidarGroundSegmenter::number_of_lowest_point_representatives(std::uint16_t number_of_lowest_point_representatives)
{
    number_of_lowest_point_representatives_ = number_of_lowest_point_representatives;
}

const double &LidarGroundSegmenter::sensor_height_offset_m() const noexcept
{
    return sensor_height_offset_m_;
}
double &LidarGroundSegmenter::sensor_height_offset_m() noexcept
{
    return sensor_height_offset_m_;
}
void LidarGroundSegmenter::sensor_height_offset_m(double sensor_height_offset_m)
{
    sensor_height_offset_m_ = sensor_height_offset_m;
}

const double &LidarGroundSegmenter::distance_threshold_m() const noexcept
{
    return distance_threshold_m_;
}
double &LidarGroundSegmenter::distance_threshold_m() noexcept
{
    return distance_threshold_m_;
}
void LidarGroundSegmenter::distance_threshold_m(double distance_threshold_m)
{
    distance_threshold_m_ = distance_threshold_m;
}

const double &LidarGroundSegmenter::initial_seed_threshold_m() const noexcept
{
    return initial_seed_threshold_m_;
}
double &LidarGroundSegmenter::initial_seed_threshold_m() noexcept
{
    return initial_seed_threshold_m_;
}
void LidarGroundSegmenter::initial_seed_threshold_m(double initial_seed_threshold_m)
{
    initial_seed_threshold_m_ = initial_seed_threshold_m;
}

Plane3D LidarGroundSegmenter::estimatePlane(const Eigen::MatrixXd &points)
{
    const auto number_of_points = points.rows();
    if (number_of_points < 3)
    {
        throw std::runtime_error("Cannot estimate plane parameters for less than three points");
    }

    // Compute centroid of the points
    Eigen::RowVector3d centroid = points.rowwise().mean();

    // Compute the deviation of points from the centroid
    Eigen::MatrixXd deviation = points.colwise() - centroid.transpose();

    // Compute the SVD of the deviation matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(deviation, Eigen::ComputeFullV);

    // Find the last column of V matrix
    Eigen::Vector3d normal = svd.matrixV().col(2);

    // Get plane parameters
    double nx = normal(0);
    double ny = normal(1);
    double nz = normal(2);
    double d = -normal.dot(centroid);

    return Plane3D{nx, ny, nz, d};
}

} // namespace segmentation