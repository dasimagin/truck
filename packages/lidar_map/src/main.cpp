#include "geom/distance.h"
#include "geom/msg.h"
#include "lidar_map/builder.h"
#include "lidar_map/conversion.h"
#include "lidar_map/serialization.h"
#include "map/map.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/program_options.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <numeric>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace truck::lidar_map;
using namespace truck::map;
using namespace truck::geom;

namespace po = boost::program_options;

namespace {

struct Metrics {
    double mean;
    double rmse;
    double q95;
    double q90;
};

/**
 * Metrics for calculations lidar map quality
 *
 * Error is calculated as distance from point cloud point
 * to the nearest segment of vector map.
 *
 * Error aggregation is done by next metrics:
 *  mean: mean error
 *  rmse: root mean squared error
 *  q95: 95th quantile
 *  q90: 90th quantile
 */
Metrics calculateMetrics(const Cloud& cloud, const ComplexPolygon& complex_polygon) {
    std::vector<double> min_dists_squared;
    const auto& segments = complex_polygon.segments();

    for (int i = 0; i < cloud.cols(); i++) {
        const Vec2 cloud_point = {cloud.col(i)(0), cloud.col(i)(1)};
        double min_dist_squared = distanceSq(cloud_point, segments[0]);

        for (size_t j = 1; j < segments.size(); j++) {
            min_dist_squared = std::min(min_dist_squared, distanceSq(cloud_point, segments[j]));
        }

        min_dists_squared.push_back(min_dist_squared);
    }

    const size_t count = min_dists_squared.size();
    std::vector<double> min_dists(count, 0);

    for (size_t i = 0; i < count; i++) {
        min_dists[i] = std::sqrt(min_dists_squared[i]);
    }

    std::sort(min_dists.begin(), min_dists.end());

    auto get_quantile = [&](double q) {
        const auto id = static_cast<size_t>(q * (count - 1));
        return min_dists[id];
    };

    Metrics metrics = {
        .mean = std::accumulate(
            min_dists.begin(),
            min_dists.end(),
            0.0,
            [&](double a, double b) { return a + b / count; }),

        .rmse = std::sqrt(std::accumulate(
            min_dists.begin(),
            min_dists.end(),
            0.0,
            [&](double a, double b) { return a + (b * b) / count; })),

        .q95 = get_quantile(0.95),
        .q90 = get_quantile(0.90)};

    return metrics;
}

std::ostream& operator<<(std::ostream& out, const Metrics& m) noexcept {
    return out << "Metrics:\n"
               << "  mean = " << m.mean << "\n"
               << "  rmse = " << m.rmse << "\n"
               << "  q95 = " << m.q95 << "\n"
               << "  q90 = " << m.q90 << "\n";
}

}  // namespace

const std::string kTopicLaserScan = "/livox/lidar";
const std::string kTopicOdom = "/ekf/odometry/filtered";
const std::string kPkgPathMap = ament_index_cpp::get_package_share_directory("map");
const std::string kPkgPathLidarMap = ament_index_cpp::get_package_share_directory("lidar_map");
const std::string kposeGraphInfoJSON = "pose_graph_info.json";

int main(int argc, char* argv[]) {
    bool enable_test = false;
    bool enable_log = false;
    std::string vector_map_file;
    std::string input_mcap_path;
    std::string output_folder_path;

    {
        po::options_description desc("Executable for constructing 2D LiDAR map");
        desc.add_options()("help,h", "show this help message and exit")(
            "input,i",
            po::value<std::string>(&input_mcap_path)->required(),
            "path to .mcap file with ride bag")(
            "output,o",
            po::value<std::string>(&output_folder_path)->required(),
            "path to folder where to store .mcap file with 2D LiDAR map (folder shouldn't exist)")(
            "test,t",
            po::value<std::string>(&vector_map_file),
            "enable test mode and set vector map (only for simulated data)")(
            "log,l", po::bool_switch(&enable_log), "enable logging");

        po::variables_map vm;
        try {
            po::store(po::parse_command_line(argc, argv, desc), vm);

            if (vm.count("help")) {
                std::cout << desc << "\n";
                return 0;
            }

            if (vm.count("test")) {
                enable_test = true;
            }

            po::notify(vm);
        } catch (po::error& e) {
            std::cerr << "ERROR: " << e.what() << "\n";
            std::cerr << desc << "\n";
            return 1;
        }
    }

    {
        const BuilderParams builder_params{
            .icp_config = kPkgPathLidarMap + "/config/icp.yaml",
            .icp_edge_max_dist = 3,
            .odom_edge_weight = 1.0,
            .icp_edge_weight = 3.0,
            .verbose = true};

        Builder builder = Builder(builder_params);

        BagWriter bag_writer = BagWriter(output_folder_path, "world", 0.5);

        Poses poses;
        Clouds clouds;

        // 1. Read data from input bag
        {
            const auto odom_msgs = loadOdomTopic(input_mcap_path, kTopicOdom);
            const auto laser_scan_msgs = loadLaserScanTopic(input_mcap_path, kTopicLaserScan);

            const auto [synced_odom_msgs, synced_laser_scan_msgs] =
                syncOdomWithCloud(odom_msgs, laser_scan_msgs);

            const auto all_poses = toPoses(synced_odom_msgs);
            const auto all_clouds = toClouds(synced_laser_scan_msgs);

            std::tie(poses, clouds) = builder.sliceDataByPosesProximity(all_poses, all_clouds, 3.0);
        }

        // 2. Construct and optimize pose graph
        {
            auto log_optimization_step = [&]() {
                auto tf_merged_clouds = builder.mergeClouds(builder.transformClouds(poses, clouds));
                bag_writer.addOptimizationStep(
                    poses, "/opt/poses", tf_merged_clouds, "/opt/clouds");
            };

            builder.initPoseGraph(poses, clouds);

            if (enable_log) {
                log_optimization_step();
            }

            const size_t optimization_steps = 10;
            if (enable_log) {
                log_optimization_step();
                const PoseGraphInfo pose_graph_info = builder.calculatePoseGraphInfo();
                const std::string pose_graph_info_path =
                    output_folder_path + "/" + kposeGraphInfoJSON;
                builder.writePoseGraphInfoToJSON(pose_graph_info_path, pose_graph_info, 0);
            }

            for (size_t i = 0; i < optimization_steps; i++) {
                poses = builder.optimizePoseGraph(1);

                if (enable_log) {
                    log_optimization_step();
                    const PoseGraphInfo pose_graph_info = builder.calculatePoseGraphInfo();
                    const std::string pose_graph_info_path =
                        output_folder_path + "/" + kposeGraphInfoJSON;
                    builder.writePoseGraphInfoToJSON(pose_graph_info_path, pose_graph_info, i + 1);
                }
            }

            clouds = builder.applyGridFilter(clouds, 0.08);

            if (enable_log) {
                log_optimization_step();
            }

            clouds = builder.applyDynamicFilter(poses, clouds, 12.0, 1, 0.05);

            if (enable_log) {
                log_optimization_step();
            }

            const auto lidar_map = builder.mergeClouds(builder.transformClouds(poses, clouds));

            bag_writer.addLidarMap(lidar_map, "/map/lidar");

            if (enable_test) {
                const std::string map_path = kPkgPathMap + "/data/" + vector_map_file;
                const ComplexPolygon vector_map = Map::fromGeoJson(map_path).polygons()[0];

                bag_writer.addVectorMap(vector_map, "/map/vector");
                std::cout << calculateMetrics(lidar_map, vector_map);
            }
        }
    }
}
