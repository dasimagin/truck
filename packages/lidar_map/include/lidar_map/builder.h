#pragma once

#include "geom/pose.h"
#include "lidar_map/common.h"

#include <g2o/core/sparse_optimizer.h>

namespace truck::lidar_map {

struct EdgeInfo {
    int from_edge;
    int to_edge;
    double error_val;
    std::string type;
};

struct PoseInfo {
    int id;
    geom::Pose pose;
};

using EdgesInfo = std::vector<EdgeInfo>;
using PosesInfo = std::vector<PoseInfo>;

struct PoseGraphInfo {
    EdgesInfo edges;
    PosesInfo poses;
};

struct BuilderParams {
    std::string icp_config;
    double icp_edge_max_dist = 0.6;
    double odom_edge_weight = 1.0;
    double icp_edge_weight = 3.0;
    bool verbose = true;
};

class Builder {
  public:
    Builder(const BuilderParams& params);

    std::pair<geom::Poses, Clouds> sliceDataByPosesProximity(
        const geom::Poses& poses, const Clouds& clouds, double poses_min_dist) const;

    void initPoseGraph(const geom::Poses& poses, const Clouds& clouds);

    geom::Poses optimizePoseGraph(size_t iterations = 1);

    PoseGraphInfo calculatePoseGraphInfo() const;

    Clouds transformClouds(
        const geom::Poses& poses, const Clouds& clouds, bool inverse = false) const;

    Cloud mergeClouds(const Clouds& clouds) const;

    Clouds applyGridFilter(const Clouds& clouds, double cell_size = 0.1) const;

    Clouds applyDynamicFilter(
        const geom::Poses& poses, const Clouds& clouds_base, double clouds_search_rad,
        size_t min_sim_points_count, double max_sim_points_dist) const;

  private:
    ICP icp_;
    g2o::SparseOptimizer optimizer_;
    BuilderParams params_;
};

}  // namespace truck::lidar_map
