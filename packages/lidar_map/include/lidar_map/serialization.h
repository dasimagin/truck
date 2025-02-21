#pragma once

#include "lidar_map/builder.h"
#include "lidar_map/common.h"
#include "geom/complex_polygon.h"

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/writer.hpp>

namespace truck::lidar_map::serialization {

using OdometryMsg = nav_msgs::msg::Odometry;
using OdometryMsgArray = std::vector<OdometryMsg>;

using PointCloudMsg = sensor_msgs::msg::PointCloud2;
using PointCloudMsgArray = std::vector<PointCloudMsg>;

std::pair<OdometryMsgArray, PointCloudMsgArray> syncOdomWithPointCloud(
    const OdometryMsgArray& odom_msgs, const PointCloudMsgArray& point_cloud_msgs);

namespace reader {

Cloud readPCD(const std::string& pcd_path);

OdometryMsgArray readOdomTopic(const std::string& mcap_path, const std::string& odom_topic);

PointCloudMsgArray readPointCloudTopic(
    const std::string& mcap_path, const std::string& point_cloud_topic);

std::pair<OdometryMsgArray, PointCloudMsgArray> readAndSyncOdomWithPointCloud(
    const std::string& mcap_path, const std::string& odom_topic,
    const std::string& point_cloud_topic, double min_odom_dist);

}  // namespace reader

namespace writer {

void writePoseGraphInfoToJSON(
    const std::string& json_path, const PoseGraphInfo& pose_graph_info, size_t iteration);

void writeToPCD(const std::string& pcd_path, const Cloud& cloud);

struct MCAPWriterParams {
    std::string mcap_path;
    std::string poses_topic_name;
    std::string cloud_topic_name;
    std::string frame_name = "";
    double topic_frequency = 1.0;
};

class MCAPWriter {
  public:
    MCAPWriter(const MCAPWriterParams& params);

    void update();

    void writeCloud(const Cloud& cloud);

    void writePoses(const geom::Poses& poses);

    static void writeCloud(
        const std::string& mcap_path, const Cloud& cloud, const std::string& topic_name,
        std::string frame_name = "");

    static void writeCloudWithAttributes(
        const std::string& mcap_path, const CloudWithAttributes& cloud_with_attributes,
        double normals_ratio = 0.5, std::string frame_name = "");

  private:
    size_t msg_id_ = 0;
    rosbag2_cpp::Writer writer_;

    MCAPWriterParams params_;
};

}  // namespace writer

}  // namespace truck::lidar_map::serialization
