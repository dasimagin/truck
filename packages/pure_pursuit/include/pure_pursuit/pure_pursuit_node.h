#pragma once

#include "pure_pursuit/pure_pursuit.h"

#include "geom/msg.h"
#include "model/model.h"
#include "truck_interfaces/msg/control.hpp"
#include "truck_interfaces/msg/trajectory.hpp"

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace truck::pure_pursuit {

class PurePursuitNode : public rclcpp::Node {
  public:
    PurePursuitNode();

  private:
    void publishCommand();

    void handleTrajectory(truck_interfaces::msg::Trajectory::SharedPtr trajectory);

    void handleOdometry(nav_msgs::msg::Odometry::SharedPtr odometry);

    rclcpp::TimerBase::SharedPtr timer_ = nullptr;

    struct State {
        std::optional<motion::Trajectory> trajectory = std::nullopt;
        std::optional<geom::Localization> localization = std::nullopt;
        nav_msgs::msg::Odometry::SharedPtr odometry = nullptr;
    } state_;

    struct Slots {
        rclcpp::Subscription<truck_interfaces::msg::Trajectory>::SharedPtr trajectory = nullptr;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry = nullptr;
    } slot_;

    struct Signals {
        rclcpp::Publisher<truck_interfaces::msg::Control>::SharedPtr command = nullptr;
    } signal_;

    std::unique_ptr<PurePursuit> controller_ = nullptr;
};

}  // namespace truck::pure_pursuit