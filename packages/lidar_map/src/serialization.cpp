#include "lidar_map/serialization.h"

#include "common/exception.h"
#include "geom/distance.h"
#include "geom/msg.h"
#include "lidar_map/conversion.h"

#include <rosbag2_cpp/reader.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <optional>

namespace truck::lidar_map::serialization {

namespace {

bool operator<(const std_msgs::msg::Header& a, const std_msgs::msg::Header& b) {
    if (a.stamp.sec < b.stamp.sec) {
        return true;
    }

    if (a.stamp.sec > b.stamp.sec) {
        return false;
    }

    return a.stamp.nanosec < b.stamp.nanosec;
}

}  // namespace

std::pair<std::vector<nav_msgs::msg::Odometry>, std::vector<sensor_msgs::msg::PointCloud2>>
syncOdomWithPointCloud(
    const std::vector<nav_msgs::msg::Odometry>& odom_msgs,
    const std::vector<sensor_msgs::msg::PointCloud2>& point_cloud_msgs) {
    VERIFY(!odom_msgs.empty());
    VERIFY(!point_cloud_msgs.empty());

    const size_t odom_count = odom_msgs.size();
    const size_t laser_scan_count = point_cloud_msgs.size();

    size_t odom_id = 0;
    size_t laser_scan_id = 0;

    std::vector<nav_msgs::msg::Odometry> odom_msgs_synced;
    std::vector<sensor_msgs::msg::PointCloud2> point_cloud_msgs_synced;

    while (laser_scan_id < laser_scan_count) {
        while (odom_id < odom_count
               && odom_msgs[odom_id].header < point_cloud_msgs[laser_scan_id].header) {
            odom_id++;
        }

        if (odom_id >= odom_count) {
            break;
        }

        odom_msgs_synced.push_back(odom_msgs[odom_id]);
        point_cloud_msgs_synced.push_back(point_cloud_msgs[laser_scan_id]);

        laser_scan_id++;
    }

    return {odom_msgs_synced, point_cloud_msgs_synced};
}

namespace reader {

Cloud readPCD(const std::string& pcd_path) {
    Cloud cloud;
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, pcl_cloud) == -1) {
        return cloud;
    }

    const size_t points_count = pcl_cloud.points.size();
    cloud = Cloud(3, points_count);

    for (size_t i = 0; i < points_count; i++) {
        cloud(0, i) = pcl_cloud.points[i].x;
        cloud(1, i) = pcl_cloud.points[i].y;
        cloud(2, i) = pcl_cloud.points[i].z;
    }

    return cloud;
}

namespace {

template<typename T>
T deserializeMessage(rosbag2_storage::SerializedBagMessageSharedPtr msg) {
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    typename T::SharedPtr ros_msg = std::make_shared<T>();
    rclcpp::Serialization<T> serialization;
    serialization.deserialize_message(&serialized_msg, ros_msg.get());
    return *ros_msg;
}

template<typename T>
std::optional<T> readNextMessage(
    std::unique_ptr<rosbag2_cpp::Reader>& reader, const std::string& topic_name) {
    while (reader->has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader->read_next();

        if (msg->topic_name != topic_name) {
            continue;
        }

        return deserializeMessage<T>(msg);
    }

    return std::nullopt;
}

}  // namespace

std::vector<nav_msgs::msg::Odometry> readOdomTopic(
    const std::string& mcap_path, const std::string& odom_topic) {
    std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();
    reader->open(mcap_path);

    std::vector<nav_msgs::msg::Odometry> data;

    while (true) {
        auto msg = readNextMessage<nav_msgs::msg::Odometry>(reader, odom_topic);

        if (!msg.has_value()) {
            break;
        }

        data.push_back(*msg);
    }

    return data;
}

std::vector<sensor_msgs::msg::PointCloud2> readPointCloudTopic(
    const std::string& mcap_path, const std::string& point_cloud_topic) {
    std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();
    reader->open(mcap_path);

    std::vector<sensor_msgs::msg::PointCloud2> data;

    while (true) {
        auto msg = readNextMessage<sensor_msgs::msg::PointCloud2>(reader, point_cloud_topic);

        if (!msg.has_value()) {
            break;
        }

        data.push_back(*msg);
    }

    return data;
}

std::pair<OdometryMsgArray, PointCloudMsgArray> readAndSyncOdomWithPointCloud(
    const std::string& mcap_path, const std::string& odom_topic,
    const std::string& point_cloud_topic, double min_odom_dist) {
    std::unique_ptr<rosbag2_cpp::Reader> reader = std::make_unique<rosbag2_cpp::Reader>();
    reader->open(mcap_path);

    OdometryMsgArray odom_msg_array;
    PointCloudMsgArray point_cloud_msg_array;

    std::optional<OdometryMsg> odom_msg = std::nullopt;
    std::optional<PointCloudMsg> point_cloud_msg = std::nullopt;

    while (reader->has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader->read_next();

        if (msg->topic_name == odom_topic) {
            odom_msg = deserializeMessage<OdometryMsg>(msg);

            if (point_cloud_msg.has_value()) {
                if (odom_msg_array.empty()) {
                    odom_msg_array.push_back(*odom_msg);
                    point_cloud_msg_array.push_back(*point_cloud_msg);
                    point_cloud_msg = std::nullopt;
                } else {
                    const double cur_odom_dist = geom::distance(
                        geom::toVec2(*odom_msg), geom::toVec2(odom_msg_array.back()));

                    if (cur_odom_dist > min_odom_dist) {
                        odom_msg_array.push_back(*odom_msg);
                        point_cloud_msg_array.push_back(*point_cloud_msg);
                        point_cloud_msg = std::nullopt;
                    } else {
                        point_cloud_msg = std::nullopt;
                    }
                }
            }
        } else if (msg->topic_name == point_cloud_topic) {
            point_cloud_msg = deserializeMessage<PointCloudMsg>(msg);
        }
    }

    return {odom_msg_array, point_cloud_msg_array};
}

}  // namespace reader

namespace writer {

/**
 * Writing information about icp edges to a json file
 */
void writePoseGraphInfoToJSON(
    const std::string& json_path, const PoseGraphInfo& pose_graph_info, size_t iteration) {
    nlohmann::json json_data;

    std::ifstream input_file(json_path);
    if (input_file.is_open()) {
        input_file >> json_data;
        input_file.close();
    }

    nlohmann::json current_iteration_data;

    for (const auto& vertex : pose_graph_info.poses) {
        nlohmann::json vertex_json;
        vertex_json["id"] = vertex.id;
        vertex_json["x"] = vertex.pose.pos.x;
        vertex_json["y"] = vertex.pose.pos.y;
        vertex_json["theta"] = vertex.pose.dir.angle().radians();

        current_iteration_data["vertices"].push_back(vertex_json);
    }

    for (const auto& edge : pose_graph_info.edges) {
        nlohmann::json edge_json;
        edge_json["from_edge"] = edge.from_edge;
        edge_json["to_edge"] = edge.to_edge;
        edge_json["error_val"] = edge.error_val;
        edge_json["type"] = edge.type;

        current_iteration_data["edges"].push_back(edge_json);
    }
    std::string formatted_iteration;

    if (iteration < 10) {
        json_data["0" + std::to_string(iteration)] = current_iteration_data;
    } else {
        json_data[std::to_string(iteration)] = current_iteration_data;
    }

    std::ofstream output_file(json_path);
    if (output_file.is_open()) {
        output_file << json_data.dump(4);
        output_file.close();
    } else {
        std::cerr << "Error when opening a file for writing: " << json_path << std::endl;
    }
}

void writeToPCD(const std::string& pcd_path, const Cloud& cloud) {
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.width = cloud.cols();
    pcl_cloud.height = 1;
    pcl_cloud.is_dense = false;
    pcl_cloud.resize(pcl_cloud.width * pcl_cloud.height);

    for (size_t i = 0; i < cloud.cols(); i++) {
        pcl_cloud.points[i].x = cloud(0, i);
        pcl_cloud.points[i].y = cloud(1, i);
        pcl_cloud.points[i].z = 1.0;
    }

    pcl::io::savePCDFileASCII(pcd_path, pcl_cloud);
}

MCAPWriter::MCAPWriter(const MCAPWriterParams& params) : params_(params) {
    if (!params.mcap_path.empty()) {
        writer_.open(params.mcap_path);
    }
}

void MCAPWriter::update() { msg_id_++; }

namespace {

rclcpp::Time getTime(double seconds = 0.0) {
    auto nanoseconds = (seconds - static_cast<int32_t>(seconds)) * 1e9;
    return {static_cast<int32_t>(seconds), static_cast<uint32_t>(nanoseconds)};
}

std_msgs::msg::ColorRGBA toColorRGBA(double a, double r, double g, double b) {
    std_msgs::msg::ColorRGBA color;
    color.a = a;
    color.r = r;
    color.g = g;
    color.b = b;
    return color;
};

geometry_msgs::msg::Vector3 toVector3(double x, double y, double z) {
    geometry_msgs::msg::Vector3 scale;
    scale.x = x;
    scale.y = y;
    scale.z = z;
    return scale;
};

}  // namespace

void MCAPWriter::writeCloud(const Cloud& cloud) {
    writer_.write(
        msg::toPointCloud2(cloud, params_.frame_name),
        params_.cloud_topic_name,
        getTime(msg_id_ * params_.topic_frequency));
}

void MCAPWriter::writePoses(const geom::Poses& poses) {
    visualization_msgs::msg::MarkerArray msg_array;

    for (size_t i = 0; i < poses.size(); i++) {
        const geom::Pose& pose = poses[i];

        visualization_msgs::msg::Marker msg;
        msg.header.frame_id = params_.frame_name;
        msg.id = i;
        msg.type = visualization_msgs::msg::Marker::ARROW;
        msg.action = visualization_msgs::msg::Marker::ADD;
        msg.color = toColorRGBA(1, 0, 0, 1);
        msg.pose.position.x = pose.pos.x;
        msg.pose.position.y = pose.pos.y;
        msg.pose.orientation = geom::msg::toQuaternion(pose.dir);
        msg.scale = toVector3(1.2, 0.2, 0.2);

        msg_array.markers.push_back(msg);
    }

    writer_.write(msg_array, params_.poses_topic_name, getTime(msg_id_ * params_.topic_frequency));
}

void MCAPWriter::writeCloud(
    const std::string& mcap_path, const Cloud& cloud, const std::string& topic_name,
    std::string frame_name) {
    rosbag2_cpp::Writer writer;
    writer.open(mcap_path);
    writer.write(msg::toPointCloud2(cloud, frame_name), topic_name, getTime());
}

void MCAPWriter::writeCloudWithAttributes(
    const std::string& mcap_path, const CloudWithAttributes& cloud_with_attributes,
    double normals_ratio, std::string frame_name) {
    rosbag2_cpp::Writer writer;
    writer.open(mcap_path);
    writer.write(msg::toPointCloud2(cloud_with_attributes.cloud, frame_name), "cloud", getTime());
    std::cout << cloud_with_attributes.normals.has_value() << ' '
              << cloud_with_attributes.weights.has_value() << '\n';
    if (cloud_with_attributes.normals.has_value()) {
        const Eigen::Matrix3Xf normals = cloud_with_attributes.normals.value();
        visualization_msgs::msg::MarkerArray msg_array;
        size_t points_count = cloud_with_attributes.cloud.cols();
        size_t step =
            static_cast<size_t>(points_count / (points_count * (1 - (normals_ratio / 100))));
        for (size_t i = 0; i < points_count; i += step) {
            visualization_msgs::msg::Marker msg_;
            msg_.header.frame_id = frame_name;
            msg_.id = i;
            msg_.type = visualization_msgs::msg::Marker::ARROW;
            msg_.action = visualization_msgs::msg::Marker::ADD;
            msg_.color = toColorRGBA(0.5, 1, 0, 0);

            msg_.pose.position.x = cloud_with_attributes.cloud(0, i);
            msg_.pose.position.y = cloud_with_attributes.cloud(1, i);
            msg_.pose.position.z = cloud_with_attributes.cloud(2, i);

            // Get direction vector components for the normal direction
            double dir_x = normals(0, i) - cloud_with_attributes.cloud(0, i);
            double dir_y = normals(1, i) - cloud_with_attributes.cloud(1, i);
            msg_.scale = toVector3(0.6, 0.06, 0.06);
            // Get yaw angle from the direction vector using atan2
            double yaw = std::atan2(dir_y, dir_x);
            msg_.pose.orientation = geom::msg::toQuaternion(truck::geom::Angle(yaw));

            msg_array.markers.push_back(msg_);
        }

        writer.write(msg_array, "normals", getTime());
    }

    if (cloud_with_attributes.weights.has_value()) {
        Cloud cloud = cloud_with_attributes.cloud;
        const Eigen::VectorXf& weights = cloud_with_attributes.weights.value();

        for (size_t i = 0; i < cloud.cols(); i++) {
            cloud(3, i) = weights(i);
        }

        writer.write(msg::toPointCloud2(cloud, frame_name), "weights", getTime());
    }
}

}  // namespace writer

}  // namespace truck::lidar_map::serialization
