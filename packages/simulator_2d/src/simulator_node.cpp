#include "simulator_node.h"

#include "simulator_2d/status_code.h"

#include "common/math.h"
#include "geom/msg.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>

namespace truck::simulator {

using namespace std::placeholders;

SimulatorNode::SimulatorNode() : Node("simulator") {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    initializeParameters();
    initializeTopicHandlers();
    initializeEngine();

    timer_ = create_wall_timer(
        std::chrono::duration<double>(params_.update_period),
        std::bind(&SimulatorNode::makeSimulationTick, this));
}

void SimulatorNode::initializeParameters() {
    params_ = {
        .update_period = declare_parameter("update_period", 0.01),

        .noise_generator =
            {.lidar =
                 {
                     .enable = declare_parameter("sensor_noise.lidar.enable", false),
                     .mean = declare_parameter("sensor_noise.lidar.mean", 0.0),
                     .variance = declare_parameter("sensor_noise.lidar.variance", 0.0),
                 },

             .gyro =
                 {
                     .enable = declare_parameter("sensor_noise.gyro.enable", false),
                     .mean = declare_parameter("sensor_noise.gyro.mean", 0.0),
                     .variance = declare_parameter("sensor_noise.gyro.variance", 0.0),
                 },

             .accel =
                 {
                     .enable = declare_parameter("sensor_noise.accel.enable", false),
                     .mean = declare_parameter("sensor_noise.accel.mean", 0.0),
                     .variance = declare_parameter("sensor_noise.accel.variance", 0.0),
                 }},

        .init_state = {
            .x = declare_parameter("initial_x", 0.0),
            .y = declare_parameter("initial_y", 0.0),
            .yaw = declare_parameter("initial_yaw", 0.0)}};
}

void SimulatorNode::initializeTopicHandlers() {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    slots_.control = Node::create_subscription<truck_msgs::msg::Control>(
        "/control/command",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&SimulatorNode::handleControl, this, _1));

    signals_.time = Node::create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(1).reliability(qos));

    signals_.localization = Node::create_publisher<nav_msgs::msg::Odometry>(
        "/simulator/localization", rclcpp::QoS(1).reliability(qos));

    signals_.hardware_odometry = Node::create_publisher<nav_msgs::msg::Odometry>(
        "/hardware/wheel/odometry", rclcpp::QoS(1).reliability(qos));

    signals_.tf_publisher =
        Node::create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(1).reliability(qos));

    signals_.telemetry = Node::create_publisher<truck_msgs::msg::HardwareTelemetry>(
        "/hardware/telemetry", rclcpp::QoS(1).reliability(qos));

    signals_.state = Node::create_publisher<truck_msgs::msg::SimulationState>(
        "/simulator/state", rclcpp::QoS(1).reliability(qos));

    signals_.scan = Node::create_publisher<sensor_msgs::msg::LaserScan>(
        "/lidar/scan", rclcpp::QoS(1).reliability(qos));

    signals_.imu = Node::create_publisher<sensor_msgs::msg::Imu>(
        "/camera/imu", rclcpp::QoS(1).reliability(qos));
}

void SimulatorNode::initializeCache(const std::unique_ptr<model::Model>& model) {
    cache_.lidar_config.angle_min = static_cast<float>(model->lidar().angle_min.radians());
    cache_.lidar_config.angle_max = static_cast<float>(model->lidar().angle_max.radians());
    cache_.lidar_config.angle_increment =
        static_cast<float>(model->lidar().angle_increment.radians());
    cache_.lidar_config.range_min = model->lidar().range_min;
    cache_.lidar_config.range_max = model->lidar().range_max;
}

void SimulatorNode::initializeEngine() {
    auto model = std::make_unique<model::Model>(
        model::load(get_logger(), declare_parameter("model_config", "")));
    initializeCache(model);

    auto noise_generator = std::make_unique<NoiseGenerator>(params_.noise_generator);

    const geom::Pose init_pose = {
        geom::Vec2(params_.init_state.x, params_.init_state.y),
        geom::AngleVec2(geom::Angle(params_.init_state.yaw))};

    engine_ = std::make_unique<SimulatorEngine>(
        std::move(model),
        std::move(noise_generator),
        declare_parameter("integration_step", 0.001),
        declare_parameter("calculations_precision", 1e-8));
    engine_->resetBase(init_pose);
    engine_->resetMap(declare_parameter("map_config", ""));

    // The zero state of the simulation.
    publishSimulationState();
}

std::optional<tf2::Transform> SimulatorNode::getLatestTranform(
    const std::string& source, const std::string& target) {
    try {
        const auto tf_msg = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
        tf2::Transform tf;
        tf2::fromMsg(tf_msg.transform, tf);
        return tf;
    } catch (const tf2::TransformException& ex) {
        return std::nullopt;
    }
}

void SimulatorNode::handleControl(const truck_msgs::msg::Control::ConstSharedPtr control) {
    if (control->has_acceleration) {
        engine_->setBaseControl(control->velocity, control->acceleration, control->curvature);
    } else {
        engine_->setBaseControl(control->velocity, control->curvature);
    }
}

void SimulatorNode::publishTime(const TruckState& truck_state) {
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = truck_state.time();
    signals_.time->publish(clock_msg);
}

void SimulatorNode::publishSimulatorLocalizationMessage(const TruckState& truck_state) {
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "world";
    msg.child_frame_id = "base";
    msg.header.stamp = truck_state.time();

    // Set the pose.
    const auto pose = truck_state.odomBasePose();
    msg.pose.pose = truck::geom::msg::toPose(pose);

    // Set the twist.
    const auto linear_velocity = truck_state.odomBaseLinearVelocity();
    msg.twist.twist.linear.x = linear_velocity.x;
    msg.twist.twist.linear.y = linear_velocity.y;
    const double angular_velocity = truck_state.baseAngularVelocity();
    msg.twist.twist.angular.z = angular_velocity;

    signals_.localization->publish(msg);
}

void SimulatorNode::publishHardwareOdometryMessage(const TruckState& truck_state) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "base";
    odom_msg.header.stamp = truck_state.time();
    odom_msg.twist.twist.linear.x = truck_state.baseTwist().velocity;
    odom_msg.twist.covariance[0] = 0.0001;
    signals_.hardware_odometry->publish(odom_msg);
}

void SimulatorNode::publishTelemetryMessage(const TruckState& truck_state) {
    truck_msgs::msg::HardwareTelemetry telemetry_msg;
    telemetry_msg.header.frame_id = "base";
    telemetry_msg.header.stamp = truck_state.time();

    const auto current_steering = truck_state.currentSteering();
    telemetry_msg.current_left_steering = current_steering.left.radians();
    telemetry_msg.current_right_steering = current_steering.right.radians();

    const auto target_steering = truck_state.targetSteering();
    telemetry_msg.target_left_steering = target_steering.left.radians();
    telemetry_msg.target_right_steering = target_steering.right.radians();

    telemetry_msg.current_rps = truck_state.currentMotorRps();
    telemetry_msg.target_rps = truck_state.targetMotorRps();

    const auto wheel_velocity = truck_state.wheelVelocity();
    telemetry_msg.rear_left_wheel_velocity = wheel_velocity.rear_left.radians();
    telemetry_msg.rear_right_wheel_velocity = wheel_velocity.rear_right.radians();
    telemetry_msg.front_left_wheel_velocity = wheel_velocity.front_left.radians();
    telemetry_msg.front_right_wheel_velocity = wheel_velocity.front_right.radians();

    signals_.telemetry->publish(telemetry_msg);
}

void SimulatorNode::publishSimulationStateMessage(const TruckState& truck_state) {
    truck_msgs::msg::SimulationState state_msg;
    state_msg.header.frame_id = "base";
    state_msg.header.stamp = truck_state.time();

    state_msg.speed = truck_state.baseTwist().velocity;
    state_msg.steering = truck_state.currentSteering().middle.radians();
    state_msg.collision = truck_state.status() == StatusCode::COLLISION;

    const auto pose = truck_state.odomBasePose();
    state_msg.pose = truck::geom::msg::toPose(pose);

    const auto angular_velocity = truck_state.gyroAngularVelocity();
    state_msg.gyro_angular_velocity.x = angular_velocity.x;
    state_msg.gyro_angular_velocity.y = angular_velocity.y;
    state_msg.gyro_angular_velocity.z = angular_velocity.z;

    const auto acceleration = truck_state.accelLinearAcceleration();
    state_msg.accel_linear_acceleration.x = acceleration.x;
    state_msg.accel_linear_acceleration.y = acceleration.y;
    state_msg.accel_linear_acceleration.z = acceleration.z;

    signals_.state->publish(state_msg);
}

void SimulatorNode::publishLaserScanMessage(const TruckState& truck_state) {
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.frame_id = "lidar_link";
    scan_msg.header.stamp = truck_state.time();
    scan_msg.angle_min = cache_.lidar_config.angle_min;
    scan_msg.angle_max = cache_.lidar_config.angle_max;
    scan_msg.angle_increment = cache_.lidar_config.angle_increment;
    scan_msg.range_min = cache_.lidar_config.range_min;
    scan_msg.range_max = cache_.lidar_config.range_max;
    scan_msg.scan_time = params_.update_period;
    scan_msg.ranges = truck_state.lidarRanges();
    signals_.scan->publish(scan_msg);
}

void SimulatorNode::publishImuMessage(const TruckState& truck_state) {
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.frame_id = "camera_imu_optical_frame";
    imu_msg.header.stamp = truck_state.time();

    // Set the sensor orientation.
    imu_msg.orientation_covariance[0] = -1;

    // Set the gyroscope.
    const auto angular_velocity = truck_state.gyroAngularVelocity();
    imu_msg.angular_velocity.x = angular_velocity.x;
    imu_msg.angular_velocity.y = angular_velocity.y;
    imu_msg.angular_velocity.z = angular_velocity.z;

    // Set the accelerometer.
    const auto acceleration = truck_state.accelLinearAcceleration();
    imu_msg.linear_acceleration.x = acceleration.x;
    imu_msg.linear_acceleration.y = acceleration.y;
    imu_msg.linear_acceleration.z = acceleration.z;

    signals_.imu->publish(imu_msg);
}

void SimulatorNode::publishSimulationState() {
    const auto truck_state = engine_->getTruckState();
    publishTime(truck_state);
    publishSimulatorLocalizationMessage(truck_state);
    publishHardwareOdometryMessage(truck_state);
    publishTelemetryMessage(truck_state);
    publishSimulationStateMessage(truck_state);
    publishLaserScanMessage(truck_state);
    publishImuMessage(truck_state);
}

void SimulatorNode::makeSimulationTick() {
    const auto tf_ekf_base = getLatestTranform("base", "odom_ekf");

    if (tf_ekf_base.has_value()) {
        transforms_.ekf_base = tf_ekf_base;
    }

    engine_->advance(params_.update_period);
    publishSimulationState();
}

}  // namespace truck::simulator
