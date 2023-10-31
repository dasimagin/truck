#include "simulator_2d/simulator_node.h"

#include "geom/msg.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

namespace truck::simulator {

using namespace std::placeholders;

SimulatorNode::SimulatorNode() : Node("simulator") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slots_.control = Node::create_subscription<truck_msgs::msg::Control>(
        "/control/command",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&SimulatorNode::handleControl, this, _1));

    signals_.time = Node::create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(1).reliability(qos));

    signals_.odometry = Node::create_publisher<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered", rclcpp::QoS(1).reliability(qos));

    signals_.tf_publisher = Node::create_publisher<tf2_msgs::msg::TFMessage>(
        "/ekf/odometry/transform", rclcpp::QoS(1).reliability(qos));

    signals_.telemetry = Node::create_publisher<truck_msgs::msg::HardwareTelemetry>(
        "/hardware/telemetry", rclcpp::QoS(1).reliability(qos));

    signals_.state = Node::create_publisher<truck_msgs::msg::SimulationState>(
        "/simulator/state", rclcpp::QoS(1).reliability(qos));

    params_ = Parameters{
        .update_period = declare_parameter("update_period", 0.01)};

    const auto model = truck::model::Model(
        declare_parameter<std::string>("model_config", "model.yaml"));
    engine_ = std::make_unique<SimulatorEngine>(
        SimulatorEngine(model, declare_parameter("integration_step", 0.001), 
        declare_parameter("calculations_precision", 1e-8)));

    // The zero state of the simulation.
    publishSimulationState();

    timer_ = create_wall_timer(
        std::chrono::duration<double>(params_.update_period),
        std::bind(&SimulatorNode::makeSimulationTick, this));    
}

void SimulatorNode::handleControl(const truck_msgs::msg::Control::ConstSharedPtr control) {
    if (control->has_acceleration) {
        engine_->setBaseControl(control->velocity, control->acceleration, control->curvature);
    } else {
        //engine_->setBaseControl(control->velocity, control->curvature);
        engine_->setBaseControl(0.1, 0);
    }
}

void SimulatorNode::publishTime() {
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = engine_->getTime();
    signals_.time->publish(clock_msg);
}

void SimulatorNode::publishOdometryMessage() {

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "odom_ekf";
    odom_msg.child_frame_id = "odom_ekf";
    odom_msg.header.stamp = engine_->getTime();

    // Set the pose.
    const auto pose = engine_->getBasePose();
    odom_msg.pose.pose.position.x = pose.pos.x;
    odom_msg.pose.pose.position.y = pose.pos.y;
    odom_msg.pose.pose.orientation = truck::geom::msg::toQuaternion(pose.dir);

    // Set the twist.
    const auto linear_velocity = engine_->getBaseLinearVelocity();
    odom_msg.twist.twist.linear.x = linear_velocity.x;
    odom_msg.twist.twist.linear.y = linear_velocity.y;
    const auto angular_velocity = engine_->getBaseAngularVelocity();
    odom_msg.twist.twist.angular.x = angular_velocity.x; 
    odom_msg.twist.twist.angular.y = angular_velocity.y;

    signals_.odometry->publish(odom_msg);
}

void SimulatorNode::publishTransformMessage() {

    geometry_msgs::msg::TransformStamped odom_to_base_transform_msg;
    odom_to_base_transform_msg.header.frame_id = "odom_ekf";
    odom_to_base_transform_msg.child_frame_id = "base";
    odom_to_base_transform_msg.header.stamp = engine_->getTime();

    // Set the transformation.
    const auto pose = engine_->getBasePose();
    odom_to_base_transform_msg.transform.translation.x = pose.pos.x;
    odom_to_base_transform_msg.transform.translation.y = pose.pos.y;
    odom_to_base_transform_msg.transform.rotation = truck::geom::msg::toQuaternion(pose.dir);

    tf2_msgs::msg::TFMessage tf_msg;
    tf_msg.transforms.push_back(odom_to_base_transform_msg);
    signals_.tf_publisher->publish(tf_msg);
}

void SimulatorNode::publishTelemetryMessage() {
    truck_msgs::msg::HardwareTelemetry telemetry_msg;
    telemetry_msg.header.frame_id = "base";
    telemetry_msg.header.stamp = engine_->getTime();
    const auto current_steering = engine_->getCurrentSteering();
    telemetry_msg.current_left_steering = current_steering.left.radians();
    telemetry_msg.current_right_steering = current_steering.right.radians();
    const auto target_steering = engine_->getTargetSteering();
    telemetry_msg.target_left_steering = target_steering.left.radians();
    telemetry_msg.target_right_steering = target_steering.right.radians();
    signals_.telemetry->publish(telemetry_msg);
}

void SimulatorNode::publishSimulationStateMessage() {
    truck_msgs::msg::SimulationState state_msg;
    state_msg.header.frame_id = "odom_ekf";
    state_msg.header.stamp = engine_->getTime();
    state_msg.speed = engine_->getBaseTwist().velocity;
    state_msg.steering = engine_->getCurrentSteering().middle.radians();
    signals_.state->publish(state_msg);
}

void SimulatorNode::publishSimulationState() {
    publishTime();
    publishOdometryMessage();
    publishTransformMessage();
    publishTelemetryMessage();
    publishSimulationStateMessage();
}

void SimulatorNode::makeSimulationTick() {
    engine_->advance(params_.update_period);
    publishSimulationState();
}

}  // namespace truck::simulator
