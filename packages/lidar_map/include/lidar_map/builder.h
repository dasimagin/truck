#pragma once

#include "geom/pose.h"
#include "lidar_map/common.h"

#include <g2o/core/sparse_optimizer.h>

namespace truck::lidar_map {

struct BuilderParams {
    std::string icp_config;
    double icp_edge_max_dist = 0.6;
    double odom_edge_weight = 1.0;
    double icp_edge_weight = 3.0;
    bool verbose = true;
};

struct EdgeInfo {
    size_t from_edge;
    size_t to_edge;
    double error_val;
    std::string type;
};

struct VertexInfo {
    size_t id;
    double x;
    double y;
    double theta;
};

using EdgesInfo = std::vector<EdgeInfo>;
using VerticesInfo = std::vector<VertexInfo>;

struct PoseGraphInfo {
    EdgesInfo edges;
    VerticesInfo vertices;

    PoseGraphInfo() : edges(), vertices() {}
};

class Builder {
  public:
    Builder(const BuilderParams& params);

    std::pair<geom::Poses, Clouds> sliceDataByPosesProximity(
        const geom::Poses& poses, const Clouds& clouds, double poses_min_dist) const;

    void initPoseGraph(const geom::Poses& poses, const Clouds& clouds);

    geom::Poses optimizePoseGraph(size_t iterations);

    PoseGraphInfo calculatePoseGraphInfo() const;

    void writePoseGraphInfoToJSON(
        const std::string& json_path, const PoseGraphInfo& pose_graph_info, size_t iteration) const;

    Clouds transformClouds(
        const geom::Poses& poses, const Clouds& clouds, bool inverse = false) const;

    Cloud mergeClouds(const Clouds& clouds) const;

    Clouds applyGridFilter(const Clouds& clouds, double cell_size) const;

    Clouds applyDynamicFilter(
        const geom::Poses& poses, const Clouds& clouds_base, double clouds_search_rad,
        size_t min_sim_points_count, double max_sim_points_dist) const;

  private:
    ICP icp_;
    g2o::SparseOptimizer optimizer_;
    BuilderParams params_;
};

}  // namespace truck::lidar_map
