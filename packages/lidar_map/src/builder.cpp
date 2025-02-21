#include "lidar_map/builder.h"

#include "common/exception.h"
#include "geom/boost/point.h"
#include "geom/boost/box.h"
#include "geom/bounding_box.h"
#include "geom/distance.h"

#include <boost/geometry.hpp>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <fstream>

namespace truck::lidar_map {

namespace bg = boost::geometry;

using IndexPoint = std::pair<geom::Vec2, size_t>;
using IndexPoints = std::vector<IndexPoint>;
using RTree = bg::index::rtree<IndexPoint, bg::index::rstar<16>>;

using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>>;
using LinearSolverType = g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

namespace {

void normalize(Cloud& cloud) {
    for (size_t i = 0; i < cloud.cols(); i++) {
        const auto scalar = cloud.col(i)(3);
        cloud.col(i) /= scalar;
    }
}

void normalize(DataPoints& data_points) {
    auto& matrix = data_points.features;
    for (size_t i = 0; i < matrix.cols(); i++) {
        const auto scalar = matrix.col(i)(3);
        matrix.col(i) /= scalar;
    }
}

DataPoints toDataPoints(const Cloud& cloud) {
    DataPoints::Labels feature_labels;
    feature_labels.push_back(DataPoints::Label("x", 1));
    feature_labels.push_back(DataPoints::Label("y", 1));
    feature_labels.push_back(DataPoints::Label("z", 1));
    feature_labels.push_back(DataPoints::Label("w", 1));

    DataPoints::Labels descriptor_labels;

    DataPoints data_points(feature_labels, descriptor_labels, cloud.cols());
    data_points.features = cloud;
    return data_points;
}

std::vector<DataPoints> toDataPoints(const Clouds& clouds) {
    std::vector<DataPoints> data_points_array;

    for (const auto& cloud : clouds) {
        data_points_array.push_back(toDataPoints(cloud));
    }

    return data_points_array;
}

/**
 * Returns g2o::SE2 constructed from pose
 */
g2o::SE2 toSE2(const geom::Pose& pose) {
    return {pose.pos.x, pose.pos.y, pose.dir.angle().radians()};
}

/**
 * Returns g2o::SE2 constructed from 4x4 transformation matrix
 */
g2o::SE2 toSE2(const Eigen::Matrix4f& tf_matrix) {
    const double tx = tf_matrix(0, 3);
    const double ty = tf_matrix(1, 3);
    const double dtheta = std::atan2(tf_matrix(1, 0), tf_matrix(0, 0));
    return {tx, ty, dtheta};
}

/**
 * Returns pose constructed from g2o::SE2
 */
geom::Pose toPose(const g2o::SE2& se2) {
    return {
        {se2.translation().x(), se2.translation().y()},
        {geom::AngleVec2::fromRadians(se2.rotation().angle())}};
}

/**
 * Returns 4x4 transformation matrix of pose relatively to a world
 *
 * Pose is given in a world frame
 *
 * Translation is taken from pose.pos
 * Rotation is taken from pose.dir
 */
Eigen::Matrix4f transformationMatrix(const geom::Pose& pose) {
    const double dtheta = pose.dir.angle().radians();
    const double cos_dtheta = std::cos(dtheta);
    const double sin_dtheta = std::sin(dtheta);

    Eigen::Matrix4f tf_matrix = Eigen::Matrix4f::Identity();

    // Rotation
    tf_matrix(0, 0) = cos_dtheta;
    tf_matrix(0, 1) = -1.0 * sin_dtheta;
    tf_matrix(1, 0) = sin_dtheta;
    tf_matrix(1, 1) = cos_dtheta;

    // Translation
    tf_matrix(0, 3) = pose.pos.x;
    tf_matrix(1, 3) = pose.pos.y;

    return tf_matrix;
}

/**
 * Returns 4x4 transformation matrix of pose_j relatively to pose_i (T_ij)
 *
 * Poses pose_i and pose_j are given in a world frame
 *
 * Translation and rotation is taken from 4x4 transformation matrix T_ij where:
 * - T_ij = T_iw * T_wj
 * - T_wj: 4x4 transformation matrix of pose_j relatively to a world
 * - T_wi: 4x4 transformation matrix of pose_i relatively to a world
 * - T_iw = (T_wi).inv(): 4x4 transformation matrix of world relatively to pose_i
 */
Eigen::Matrix4f transformationMatrix(const geom::Pose& pose_i, const geom::Pose& pose_j) {
    const Eigen::Matrix4f T_wj = transformationMatrix(pose_j);
    const Eigen::Matrix4f T_wi = transformationMatrix(pose_i);
    return T_wi.inverse() * T_wj;
}

}  // namespace

Builder::Builder(const BuilderParams& params) : params_(params) {
    std::ifstream icp_config_stream(params_.icp_config);
    icp_.loadFromYaml(icp_config_stream);
}

/**
 * Returns a subset of given poses and corresponding clouds via removing poses which are too close
 * Next pose will be added, if it's far enough from a previous one
 */
std::pair<geom::Poses, Clouds> Builder::sliceDataByPosesProximity(
    const geom::Poses& poses, const Clouds& clouds, double poses_min_dist) const {
    VERIFY(!poses.empty());
    VERIFY(poses.size() == clouds.size());

    geom::Poses filtered_poses = {poses[0]};
    Clouds filtered_clouds = {clouds[0]};

    size_t last_added_pose_index = 0;
    for (size_t i = 1; i < poses.size(); i++) {
        const double dist = geom::distance(poses[i].pos, poses[last_added_pose_index].pos);

        if (dist < poses_min_dist) {
            continue;
        }

        filtered_poses.push_back(poses[i]);
        filtered_clouds.push_back(clouds[i]);
        last_added_pose_index = i;
    }

    return {filtered_poses, filtered_clouds};
}

/**
 * Initialize pose graph
 *
 * Input:
 * - 'poses': set of clouds' poses in a world frame
 * - 'clouds': set of clouds in correspondig local frames
 */
void Builder::initPoseGraph(const geom::Poses& poses, const Clouds& clouds) {
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    optimizer_.clear();
    optimizer_.setAlgorithm(solver);

    std::vector<g2o::VertexSE2*> vertices;

    // Add vertices
    for (size_t i = 0; i < poses.size(); i++) {
        auto* vertex = new g2o::VertexSE2();
        vertex->setId(i);
        vertex->setEstimate(toSE2(poses[i]));

        optimizer_.addVertex(vertex, 0);
        vertices.push_back(vertex);
    }

    // Add odometry edges
    for (size_t i = 1; i < poses.size(); i++) {
        const Eigen::Matrix4f tf_matrix_odom = transformationMatrix(poses[i - 1], poses[i]);

        auto* edge = new g2o::EdgeSE2();
        edge->setVertex(0, vertices[i - 1]);
        edge->setVertex(1, vertices[i]);
        edge->setMeasurement(toSE2(tf_matrix_odom));
        edge->setInformation(Eigen::Matrix3d::Identity() * params_.odom_edge_weight);
        auto* userData = new EdgeData(0);
        edge->setUserData(userData);

        optimizer_.addEdge(edge);
    }

    auto data_points_clouds = toDataPoints(clouds);

    // Add ICP edges
    for (size_t i = 0; i < poses.size(); i++) {
        for (size_t j = i + 1; j < poses.size(); j++) {
            if (geom::distance(poses[i], poses[j]) > params_.icp_edge_max_dist || 
            std::abs(static_cast<int>(i) - static_cast<int>(j)) < 12) {
            continue;
        }

            const Eigen::Matrix4f tf_matrix_odom = transformationMatrix(poses[i], poses[j]);

            const auto& reference_cloud = data_points_clouds[i];
            auto reading_cloud = data_points_clouds[j];
            icp_.transformations.apply(reading_cloud, tf_matrix_odom);
            normalize(reading_cloud);
            const Eigen::Matrix4f tf_matrix_icp = icp_(reading_cloud, reference_cloud);
            const Eigen::Matrix4f tf_matrix_final = tf_matrix_icp * tf_matrix_odom;
            auto* edge = new g2o::EdgeSE2();
            edge->setVertex(0, vertices[i]);
            edge->setVertex(1, vertices[j]);
            edge->setMeasurement(toSE2(tf_matrix_final));
            edge->setInformation(Eigen::Matrix3d::Identity() * params_.icp_edge_weight);
            auto* userData = new EdgeData(1);
            edge->setUserData(userData);

            optimizer_.addEdge(edge);
        }
    }

    auto* fixed_vertex = dynamic_cast<g2o::VertexSE2*>(optimizer_.vertex(0));
    fixed_vertex->setFixed(true);

    optimizer_.setVerbose(params_.verbose);
    optimizer_.initializeOptimization();
}

/**
 * Do pose graph optimization
 *
 * This functions should be called only after 'initPoseGraph()' function
 *
 * Output:
 * - set of optimized clouds' poses in a world frame
 */
geom::Poses Builder::optimizePoseGraph(size_t iterations) {
    optimizer_.optimize(iterations);

    if (params_.verbose) {
        std::cout << "[LOG] optimizePoseGraph(): finished" << std::endl;
    }

    geom::Poses optimized_poses;

    for (size_t i = 0; i < optimizer_.vertices().size(); i++) {
        auto* optimized_vertex = dynamic_cast<g2o::VertexSE2*>(optimizer_.vertex(i));
        if (optimized_vertex) {
            const g2o::SE2 se2 = optimized_vertex->estimate();
            optimized_poses.push_back(toPose(se2));
        }
    }

    return optimized_poses;
}

/**
 * Collecting information about ICP edges
 */
PoseGraphInfo Builder::calculatePoseGraphInfo() const {
    PoseGraphInfo pose_graph_info;
    for (auto it = optimizer_.activeEdges().begin(); it != optimizer_.activeEdges().end(); ++it) {
        const g2o::OptimizableGraph::Edge* edge = *it;
        const g2o::EdgeSE2* edge_se2 = dynamic_cast<const g2o::EdgeSE2*>(edge);
        const g2o::OptimizableGraph::Vertex* from_edge =
            dynamic_cast<const g2o::OptimizableGraph::Vertex*>(edge_se2->vertex(0));
        const g2o::OptimizableGraph::Vertex* to_edge =
            dynamic_cast<const g2o::OptimizableGraph::Vertex*>(edge_se2->vertex(1));

        EdgeData* myDataPtr =
            dynamic_cast<EdgeData*>(const_cast<g2o::HyperGraph::Data*>(edge->userData()));
        const bool is_icp_edge = (myDataPtr != nullptr && myDataPtr->getValue() == 1);

        EdgeInfo edge_info = {
            .from_edge = from_edge->id(),
            .to_edge = to_edge->id(),
            .error_val = edge->chi2(),
            .type = is_icp_edge ? "icp" : "odom"};

        pose_graph_info.edges.push_back(edge_info);
    }
    for (auto it = optimizer_.activeVertices().begin(); it != optimizer_.activeVertices().end();
         ++it) {
        const g2o::OptimizableGraph::Vertex* vertex = *it;
        const g2o::VertexSE2* vertex_se2 = dynamic_cast<const g2o::VertexSE2*>(vertex);
        Eigen::Vector3d estimate;
        vertex_se2->getEstimateData(estimate.data());
        pose_graph_info.poses.push_back(PoseInfo{
            .id = vertex_se2->id(),
            .pose = {
                .pos = geom::Vec2{estimate[0], estimate[1]},
                .dir = truck::geom::Angle::fromRadians(estimate[2])}});
    }
    return pose_graph_info;
}

/**
 * Transform points' coordinates of each cloud
 *
 * Input:
 * - 'poses': set of clouds' poses in a world frame
 * - 'clouds': set of clouds in a world frame / corresponding local frames
 * - 'inverse' (false): from corresponding local frames to a common world frame
 * - 'inverse' (true): from a common world frame to corresponding local frames
 *
 * Output:
 * - set of clouds in a world frame / corresponding local frames
 */
Clouds Builder::transformClouds(
    const geom::Poses& poses, const Clouds& clouds, bool inverse) const {
    VERIFY(!poses.empty());
    VERIFY(poses.size() == clouds.size());

    Clouds clouds_tf;

    for (size_t i = 0; i < clouds.size(); i++) {
        Eigen::Matrix4f tf_matrix = transformationMatrix(poses[i]);
        tf_matrix = (inverse == true) ? tf_matrix.inverse() : tf_matrix;

        Cloud cloud_tf = tf_matrix * clouds[i];
        normalize(cloud_tf);

        clouds_tf.push_back(cloud_tf);
    }

    return clouds_tf;
}

/**
 * Merge clouds column-wise
 */
Cloud Builder::mergeClouds(const Clouds& clouds) {
    VERIFY(!clouds.empty());
    size_t points_count = 0;

    for (const auto& cloud : clouds) {
        points_count += cloud.cols();
    }

    Cloud merged_cloud(4, points_count);
    size_t last_point_id = 0;

    for (const auto& cloud : clouds) {
        merged_cloud.block(0, last_point_id, cloud.rows(), cloud.cols()) = cloud;
        last_point_id += cloud.cols();
    }

    return merged_cloud;
}

/**
 * get outliers weights for cloud
 */
Eigen::VectorXf Builder::calculateWeightsForReadingCloud(
    const Cloud& reading_cloud, const Cloud& reference_cloud) {
    DataPoints reference_dp = toDataPoints(reference_cloud);
    DataPoints reading_dp = toDataPoints(reading_cloud);

    icp_.referenceDataPointsFilters.apply(reference_dp);
    icp_.matcher->init(reference_dp);
    Matcher::Matches matches = icp_.matcher->findClosests(reading_dp);
    Matcher::OutlierWeights outlierWeights =
        icp_.outlierFilters.compute(reading_dp, reference_dp, matches);

    Eigen::VectorXf weights(outlierWeights.cols());
    for (size_t i = 0; i < outlierWeights.cols(); i++) {
        weights(i) = outlierWeights(0, i);
    }

    return weights;
}

/**
 * get normals for cloud
 */
Eigen::Matrix3Xf Builder::calculateNormalsForReferenceCloud(const Cloud& reference_cloud) {
    DataPoints reference_dp = toDataPoints(reference_cloud);
    icp_.referenceDataPointsFilters.apply(reference_dp);
    return reference_dp.getDescriptorViewByName("normals");
}

namespace {

std::vector<size_t> findNearestIdsInsideBox(
    const RTree& rtree, const IndexPoint& query_index_point, double search_rad) {
    IndexPoints index_points;

    const geom::Vec2& query_point = query_index_point.first;
    const size_t query_index = query_index_point.second;

    const geom::BoundingBox bbox(
        {query_point.x - search_rad, query_point.y - search_rad},
        {query_point.x + search_rad, query_point.y + search_rad});

    rtree.query(bg::index::intersects(bbox), std::back_inserter(index_points));

    std::vector<size_t> ids;

    for (const auto& index_point : index_points) {
        if (index_point.second != query_index) {
            ids.push_back(index_point.second);
        }
    }

    return ids;
}

geom::Vec2 findNearestPoint(const RTree& rtree, const geom::Vec2& point) {
    IndexPoints index_points;
    rtree.query(bg::index::nearest(point, 1), std::back_inserter(index_points));
    return index_points.back().first;
}

RTree toRTree(const geom::Poses& poses) {
    RTree rtree;
    for (size_t i = 0; i < poses.size(); i++) {
        rtree.insert({poses[i].pos, i});
    }
    return rtree;
}

RTree toRTree(const Cloud& cloud) {
    RTree rtree;
    for (size_t i = 0; i < cloud.cols(); i++) {
        const geom::Vec2 cloud_point = {cloud(0, i), cloud(1, i)};
        rtree.insert({cloud_point, i});
    }
    return rtree;
}

}  // namespace

/**
 * Down-sample clouds by removing rare (dynamic) points
 *
 * Input:
 * - 'poses': set of clouds' poses in a world frame
 * - 'clouds_base': set of clouds in corresponding local frames
 *
 * Output:
 * - set of filtered clouds in corresponding local frames
 *
 * In every i-th cloud we look through every j-th point, let's refer to it as a reference point
 *
 * Reference point will not be deleted from i-th cloud if the following condition is met:
 * - for a reference point, we must find at least 'min_sim_points_count' similar points
 *   among nearest clouds which are located in 'clouds_search_rad' radius reletively to i-th cloud.
 *   Point is considered similar to a reference point if it's located no further than
 *   'max_sim_points_dist' meters away from a reference point
 */
Clouds Builder::applyDynamicFilter(
    const geom::Poses& poses, const Clouds& clouds_base, double clouds_search_rad,
    size_t min_sim_points_count, double max_sim_points_dist) const {
    VERIFY(!poses.empty());
    VERIFY(poses.size() == clouds_base.size());

    // Make a transformation of clouds' points from corresponding local frames
    // defined by clouds' poses into a world frame
    const Clouds clouds = transformClouds(poses, clouds_base, false);

    // Build rtree for poses
    const RTree poses_rtree = toRTree(poses);

    // Build rtree for every cloud
    std::vector<RTree> clouds_rtrees;
    for (const auto& cloud : clouds) {
        clouds_rtrees.push_back(toRTree(cloud));
    }

    using CloudSkeleton = std::vector<size_t>;
    std::vector<CloudSkeleton> clouds_skeletons(clouds.size());

    for (size_t cloud_id = 0; cloud_id < clouds.size(); cloud_id++) {
        if (params_.verbose) {
            std::cout << "[LOG] applyDynamicFilter(): "
                      << "iteration " << cloud_id << " of " << clouds.size() << ".\n";
        }

        const std::vector<size_t> nearest_clouds_ids = findNearestIdsInsideBox(
            poses_rtree, {poses[cloud_id].pos, cloud_id}, clouds_search_rad);

        for (size_t point_id = 0; point_id < clouds[cloud_id].cols(); point_id++) {
            const geom::Vec2 cur_point = {
                clouds[cloud_id](0, point_id), clouds[cloud_id](1, point_id)};

            size_t cur_sim_points_count = 0;

            for (size_t neighbor_cloud_id : nearest_clouds_ids) {
                if (cur_sim_points_count == min_sim_points_count) {
                    break;
                }

                const geom::Vec2 neighbor_cloud_point =
                    findNearestPoint(clouds_rtrees[neighbor_cloud_id], cur_point);

                if (geom::distance(cur_point, neighbor_cloud_point) < max_sim_points_dist) {
                    cur_sim_points_count++;
                }
            }

            if (cur_sim_points_count == min_sim_points_count) {
                clouds_skeletons[cloud_id].push_back(point_id);
            }
        }
    }

    // Output
    Clouds filtered_clouds;

    for (size_t i = 0; i < clouds.size(); i++) {
        if (clouds_skeletons[i].size() == 0) {
            std::cout << "[WARNING] applyDynamicFilter(): "
                      << "one of filtered cloud is now empty, "
                      << "because of too strong filtering params, try to change them, "
                      << "for now this function will return default clouds.\n";
            return clouds_base;
        }

        Cloud filtered_cloud(4, clouds_skeletons[i].size());

        size_t last_point_id = 0;
        for (size_t point_id : clouds_skeletons[i]) {
            filtered_cloud.block(0, last_point_id, 4, 1) = clouds[i].col(point_id);
            last_point_id++;
        }

        filtered_clouds.push_back(filtered_cloud);
    }

    // Make a transformation of clouds' points from a world frame
    // into corresponding local frames defined by clouds' poses
    return transformClouds(poses, filtered_clouds, true);
}

Clouds Builder::applyBoundingBoxFilter(const Clouds& clouds, double value) const {
    const std::string value_str = std::to_string(value);
    const std::string neg_value_str = std::to_string(-value);

    PointMatcherSupport::Parametrizable::Parameters bbox_filter_params = {
        {"xMin", neg_value_str},
        {"xMax", value_str},
        {"yMin", neg_value_str},
        {"yMax", value_str},
        {"zMin", neg_value_str},
        {"zMax", value_str},
    };

    std::shared_ptr<Matcher::DataPointsFilter> bbox_filter =
        Matcher::get().DataPointsFilterRegistrar.create(
            "BoundingBoxDataPointsFilter", bbox_filter_params);

    Clouds clouds_filtered;

    for (const auto& cloud : clouds) {
        clouds_filtered.push_back(bbox_filter->filter(toDataPoints(cloud)).features);
    }

    return clouds_filtered;
}


/**
 * Down-sample clouds by taking a spatial average of clouds's points
 *
 * As we work in 2D case, we divide the plane into a regular grid of rectangles,
 * sampling rate is adjusted by setting the grid cell size along each dimension
 *
 * The set of points which lie within the bounds of a grid cell are combined into one output point
 */
Clouds Builder::applyGridFilter(const Clouds& clouds, double cell_size) const {
    const std::string cell_size_str = std::to_string(cell_size);
    PointMatcherSupport::Parametrizable::Parameters grid_filter_params = {
        {"vSizeX", cell_size_str},
        {"vSizeY", cell_size_str},
        {"vSizeZ", cell_size_str},
    };

    std::shared_ptr<Matcher::DataPointsFilter> grid_filter =
        Matcher::get().DataPointsFilterRegistrar.create(
            "VoxelGridDataPointsFilter", grid_filter_params);

    Clouds clouds_filtered;

    for (const auto& cloud : clouds) {
        clouds_filtered.push_back(grid_filter->filter(toDataPoints(cloud)).features);
    }

    return clouds_filtered;
}

Cloud Builder::applyGridFilter(const Cloud& cloud, double cell_size) const {
    std::cout << "1\n";
    const std::string cell_size_str = std::to_string(cell_size);
    std::cout << "2\n";
    PointMatcherSupport::Parametrizable::Parameters grid_filter_params = {
        {"vSizeX", cell_size_str},
        {"vSizeY", cell_size_str},
        {"vSizeZ", cell_size_str},
    };
    std::cout << "3\n";
    std::shared_ptr<Matcher::DataPointsFilter> grid_filter =
        Matcher::get().DataPointsFilterRegistrar.create(
            "VoxelGridDataPointsFilter", grid_filter_params);
    std::cout << "4\n";
    std::cout << cloud.cols() << ' ' << cloud.rows() << '\n';
    auto dp = toDataPoints(cloud);
    std::cout << "5\n";
    std::cout << dp.features.cols() << ' ' << dp.features.rows() << '\n';
    auto s = grid_filter->filter(dp).features;
    std::cout << "6\n";
    Cloud result = grid_filter->filter(toDataPoints(cloud)).features;
    return result;
}

}  // namespace truck::lidar_map
