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

std::uint8_t LidarGroundSegmenter::numberOfIterations() const noexcept
{
    return number_of_iterations_;
}
void LidarGroundSegmenter::numberOfIterations(std::uint8_t number_of_iterations) noexcept
{
    number_of_iterations_ = number_of_iterations;
}

std::uint8_t LidarGroundSegmenter::numberOfPlanarPartitions() const noexcept
{
    return number_of_planar_partitions_;
}
void LidarGroundSegmenter::numberOfPlanarPartitions(std::uint8_t number_of_planar_partitions) noexcept
{
    number_of_planar_partitions_ = number_of_planar_partitions;
}

std::uint16_t LidarGroundSegmenter::numberOfLowestPointRepresentatives() const noexcept
{
    return number_of_lowest_point_representatives_;
}
void LidarGroundSegmenter::numberOfLowestPointRepresentatives(
    std::uint16_t number_of_lowest_point_representatives) noexcept
{
    number_of_lowest_point_representatives_ = number_of_lowest_point_representatives;
}

double LidarGroundSegmenter::sensorHeightOffsetM() const noexcept
{
    return sensor_height_offset_m_;
}
void LidarGroundSegmenter::sensorHeightOffsetM(double sensor_height_offset_m) noexcept
{
    sensor_height_offset_m_ = sensor_height_offset_m;
}

double LidarGroundSegmenter::distanceThresholdM() const noexcept
{
    return distance_threshold_m_;
}
void LidarGroundSegmenter::distanceThresholdM(double distance_threshold_m) noexcept
{
    distance_threshold_m_ = distance_threshold_m;
}

double LidarGroundSegmenter::initialSeedThresholdM() const noexcept
{
    return initial_seed_threshold_m_;
}
void LidarGroundSegmenter::initialSeedThresholdM(double initial_seed_threshold_m) noexcept
{
    initial_seed_threshold_m_ = initial_seed_threshold_m;
}

std::optional<Plane3D> LidarGroundSegmenter::estimatePlane(const std::vector<IndexedPoint3D> &points) noexcept
{
    // Need at least 3 points to form a plane
    const std::size_t number_of_points = points.size();
    if (number_of_points < 3)
    {
        return {};
    }

    // Create an Eigen::MatrixXd with the same dimensions as the input vector
    Eigen::MatrixXd point_matrix(number_of_points, 3);

    // Copy data from the vector to the matrix
    for (std::size_t row = 0; row < number_of_points; ++row)
    {
        const auto &point = points[row];
        point_matrix(row, 0) = point.x;
        point_matrix(row, 1) = point.y;
        point_matrix(row, 2) = point.z;
    }

    // Compute centroid of the points
    Eigen::RowVector3d centroid = point_matrix.colwise().mean();

    // Compute the deviation of points from the centroid
    Eigen::MatrixXd deviation = point_matrix.rowwise() - centroid;

    // Compute the SVD of the deviation matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(deviation, Eigen::ComputeFullV);

    // Find the last column of V matrix
    Eigen::Vector3d normal = svd.matrixV().col(2);

    // Get plane parameters
    double nx = normal(0);
    double ny = normal(1);
    double nz = normal(2);
    double d = -normal.dot(centroid);

    return std::make_optional<Plane3D>(nx, ny, nz, d);
}

void LidarGroundSegmenter::formPlanarPartitions(const std::vector<IndexedPoint3D> &points,
                                                std::vector<std::vector<IndexedPoint3D>> &segments)
{
}
} // namespace segmentation