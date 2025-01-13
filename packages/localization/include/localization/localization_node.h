#pragma once

#include "geom/vector.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pointmatcher/PointMatcher.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace truck::localization {

using Matcher = PointMatcher<float>;
using ICP = Matcher::ICP;
using DataPoints = Matcher::DataPoints;

struct LocalizationParams {
    std::chrono::duration<double> period;
    bool verbose;
    std::string icp_config;

    struct BBoxFilter {
        bool enable;
        double radius;
        double z_min;
        double z_max;
    } bbox_filter;

    struct LocalScan {
        struct Rendering {
            struct BBoxFiltered {
                bool enable;
            } bbox_filtered;
        } rendering;
    } local_scan;

    struct GlobalScan {
        std::string config;

        struct Rendering {
            struct Main {
                bool enable;
                std::chrono::duration<double> period;
            } main;

            struct BBoxFiltered {
                bool enable;
            } bbox_filtered;
        } rendering;
    } global_scan;
};

class LocalizationNode : public rclcpp::Node {
  public:
    LocalizationNode();

  private:
    void initializeParams();
    void initializeTopicHandlers();

    void loadScanGlobal();

    void onReset(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onLocalScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    void makeLocalizationTick();

    void publishTf();
    void publishScanGlobal();

    std::optional<tf2::Transform> getLatestTranform(
        const std::string& source, const std::string& target);

    struct Signals {
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_scan = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_scan_bbox_filtered =
            nullptr;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_scan_bbox_filtered =
            nullptr;
    } signals_;

    struct Slots {
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr local_scan = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose = nullptr;
    } slots_;

    struct Timers {
        rclcpp::TimerBase::SharedPtr main = nullptr;
        rclcpp::TimerBase::SharedPtr global_scan = nullptr;
    } timers_;

    struct Scans {
        struct Global {
            DataPoints data_points;
            sensor_msgs::msg::PointCloud2 point_cloud;
        } global;
        sensor_msgs::msg::LaserScan local;
    } scans_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;

    ICP icp_;
    tf2::Transform tf_world_ekf_;

    LocalizationParams params_;
};

}  // namespace truck::localization
