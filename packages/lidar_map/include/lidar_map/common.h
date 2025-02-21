#pragma once

#include <pointmatcher/PointMatcher.h>
#include <optional>

namespace truck::lidar_map {

using Matcher = PointMatcher<float>;
using ICP = Matcher::ICP;
using DataPoints = Matcher::DataPoints;

/**
 * 4xn eigen matrix of 3D point cloud in homogeneous coordinates
 */
using Cloud = Eigen::Matrix4Xf;

/**
 * 4xn eigen matrices of 3D point clouds in homogeneous coordinates
 */
using Clouds = std::vector<Cloud>;

struct CloudWithAttributes {
    Cloud cloud;
    std::optional<Eigen::VectorXf> weights = std::nullopt;

    // normals_x, normals_y, normals_z - components of the normal vector that indicate the direction
    // perpendicular to the surface passing through the point.
    std::optional<Eigen::Matrix3Xf> normals = std::nullopt;
};

}  // namespace truck::lidar_map
