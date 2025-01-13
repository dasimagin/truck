#pragma once

#include <pointmatcher/PointMatcher.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using Matcher = PointMatcher<float>;
using DataPoints = Matcher::DataPoints;

namespace truck::localization {

DataPoints toDataPoints(const sensor_msgs::msg::LaserScan& msg);

DataPoints toDataPoints(const sensor_msgs::msg::PointCloud2& msg);

sensor_msgs::msg::PointCloud2 toPointCloud2(
    const std_msgs::msg::Header& header, const DataPoints& data_points);

}  // namespace truck::localization
