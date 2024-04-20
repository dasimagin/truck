#pragma once

#include "common/math.h"
#include "geom/angle.h"
#include "model/params.h"

#include <tf2_ros/buffer.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <utility>
#include <string>

namespace truck::model {

struct Steering {
    geom::Angle middle;
    geom::Angle left;
    geom::Angle right;
};

struct WheelVelocity {
    geom::Angle left;
    geom::Angle right;
};

struct Twist {
  double curvature;
  double velocity;
};

class Model {
  public:
    Model(const std::string& config_path);

    // Limits
    double baseMaxAbsCurvature() const;
    double steeringVelocity() const;
    double baseMaxAcceleration() const;
    double baseMaxDeceleration() const;
    Limits<geom::Angle> leftSteeringLimits() const;
    Limits<geom::Angle> rightSteeringLimits() const;
    Limits<double> middleSteeringLimits() const;
    Limits<double> baseVelocityLimits() const;
    Limits<double> baseCurvatureLimits() const;
    ServoAngles servoHomeAngles() const;

    double gearRatio() const;

    const Shape& shape() const;
    const WheelBase& wheelBase() const;
    const Wheel& wheel() const;
    const Lidar& lidar() const;

    /**
     * @param rear_twist Twist of the center of the rear axle.
     * @param rear_to_point Translation from the center of the rear axle to the arbitrary point.
     *
     * @return Twist of the arbitrary point.
     * 
     * Truck scheme:
     * -------
     * |     |
     * | X   |
     * |  C  |
     * |     |
     * |     |
     * ---R---
     * C - the center of the base.
     * R - the center of the rear axle.
     * X - the arbitrary point.
     * rear_to_point - a vector RX.
     */
    Twist rearToArbitraryPointTwist(Twist rear_twist, const geom::Vec2& rear_to_point) const;
    Twist baseToRearTwist(Twist base_twist) const;
    Twist rearToBaseTwist(Twist rear_twist) const;
    Steering rearTwistToSteering(Twist rear_twist) const;
    Steering rearCurvatureToSteering(double curvature) const;
    double middleSteeringToRearCurvature(double steering) const;
    double baseToRearAcceleration(double acceleration, double base_curvature) const;
    WheelVelocity rearTwistToWheelVelocity(Twist rear_twist) const;
    double linearVelocityToMotorRPS(double velocity) const;
    double motorRPStoLinearVelocity(double rps) const;

    tf2_msgs::msg::TFMessage getTfStaticMsg() const;
    tf2::Transform getLatestTranform(const std::string& source, 
      const std::string& target) const;

  private:
    struct Cache {
        double width_half;
        double max_abs_curvature;
        Limits<double> middle_steering_limits;
        Limits<double> base_curvature_limits;
        tf2_msgs::msg::TFMessage tf_static_msg;
        std::shared_ptr<tf2_ros::Buffer> tf_static_buffer;
    } cache_;

    Params params_;
};

inline auto load(rclcpp::Logger logger, const std::string& path) {
    RCLCPP_INFO(logger, "load model: %s", path.c_str());
    return Model(path);
}

inline auto makeUniquePtr(rclcpp::Logger logger, const std::string& path) {
    RCLCPP_INFO(logger, "load model: %s", path.c_str());
    return std::make_unique<Model>(path);
}

}  // namespace truck::model