#ifndef LIDAR_GROUND_SEGMENTER_HPP
#define LIDAR_GROUND_SEGMENTER_HPP

#include <algorithm>
#include <array>
#include <cstdint>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <execution>
#include <memory>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <vector>

namespace segmentation
{
using Point3D = std::array<double, 3>;

struct IndexedPoint3D
{
    IndexedPoint3D() = default;
    IndexedPoint3D(double x, double y, double z, std::size_t index);
    double x;
    double y;
    double z;
    std::size_t index;
};

struct Plane3D
{
    Plane3D() = default;
    Plane3D(double nx, double ny, double nz, double d);
    Eigen::Vector3d normal;
    double d;
};

class LidarGroundSegmenter final
{
  public:
    LidarGroundSegmenter() = delete;
    LidarGroundSegmenter(const LidarGroundSegmenter &) = delete;
    LidarGroundSegmenter &operator=(const LidarGroundSegmenter &) = delete;
    LidarGroundSegmenter(LidarGroundSegmenter &&) = default;
    LidarGroundSegmenter &operator=(LidarGroundSegmenter &&) = default;
    ~LidarGroundSegmenter() = default;
    LidarGroundSegmenter(const std::vector<Point3D> &points, std::uint8_t number_of_iterations = 3,
                         std::uint8_t number_of_planar_partitions = 1,
                         std::uint16_t number_of_lowest_point_representatives = 400,
                         double sensor_height_offset_m = 1.73, double distance_threshold_m = 0.3,
                         double initial_seed_threshold_m = 0.6);

    std::uint8_t numberOfIterations() const noexcept;
    void numberOfIterations(std::uint8_t number_of_iterations) noexcept;

    std::uint8_t numberOfPlanarPartitions() const noexcept;
    void numberOfPlanarPartitions(std::uint8_t number_of_planar_partitions) noexcept;

    std::uint16_t numberOfLowestPointRepresentatives() const noexcept;
    void numberOfLowestPointRepresentatives(std::uint16_t number_of_lowest_point_representatives) noexcept;

    double sensorHeightOffsetM() const noexcept;
    void sensorHeightOffsetM(double sensor_height_offset_m) noexcept;

    double distanceThresholdM() const noexcept;
    void distanceThresholdM(double distance_threshold_m) noexcept;

    double initialSeedThresholdM() const noexcept;
    void initialSeedThresholdM(double initial_seed_threshold_m) noexcept;

    std::optional<Plane3D> estimatePlane(const std::vector<IndexedPoint3D> &points) noexcept;

  private:
    void formPlanarPartitions(const std::vector<IndexedPoint3D> &points,
                              std::vector<std::vector<IndexedPoint3D>> &segments);

    std::uint8_t number_of_iterations_;
    std::uint8_t number_of_planar_partitions_;
    std::uint16_t number_of_lowest_point_representatives_;
    double sensor_height_offset_m_;
    double distance_threshold_m_;
    double initial_seed_threshold_m_;

    std::vector<IndexedPoint3D> processing_points_;
    std::vector<std::uint8_t> labels_;
};
} // namespace segmentation

#endif // LIDAR_GROUND_SEGMENTER_HPP