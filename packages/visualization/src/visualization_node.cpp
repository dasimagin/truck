#include "visualization_node.h"
#include "color.h"

#include "common/math.h"
#include "geom/angle.h"
#include "geom/arc.h"
#include "geom/msg.h"
#include "geom/pose.h"
#include "geom/segment.h"

#include <boost/assert.hpp>

#include <functional>
#include <utility>

namespace truck::visualization {

using namespace std::placeholders;

using namespace geom::literals;

VisualizationNode::VisualizationNode() : Node("visualization") {
    model_ = model::makeUniquePtr(
        this->get_logger(),
        Node::declare_parameter<std::string>("model_config", "model.yaml"));

    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    params_ = Parameters{
        .ttl = rclcpp::Duration::from_seconds(this->declare_parameter("ttl", 1.0)),

        .ego_z_lev = this->declare_parameter("ego.z_lev", 0.0),
        .ego_height = this->declare_parameter("ego.height", 0.2),

        .ego_track_width = this->declare_parameter("ego.track.width", 0.06),
        .ego_track_height = this->declare_parameter("ego.track.height", 0.01),
        .ego_track_ttl =
            rclcpp::Duration::from_seconds(this->declare_parameter("ego.track.ttl", 2.00)),

        .arc_z_lev = this->declare_parameter("arc.z_lev", 0.0),
        .arc_width = this->declare_parameter("arc.width", 0.06),
        .arc_length = this->declare_parameter("arc.length", 1.0),

        .waypoints_z_lev = this->declare_parameter("waypoints.z_lev", 0.50),
        .waypoints_radius = this->declare_parameter("waypoints.radius", 0.10),

        .trajectory_z_lev = this->declare_parameter("trajectory.z_lev", 0.0),
        .trajectory_width = this->declare_parameter("trajector.width", 0.12),
    };

    slot_.mode = Node::create_subscription<truck_msgs::msg::ControlMode>(
        "/control/mode",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleMode, this, _1));

    slot_.control = Node::create_subscription<truck_msgs::msg::Control>(
        "/control/command",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleControl, this, _1));

    slot_.waypoints = Node::create_subscription<truck_msgs::msg::Waypoints>(
        "/waypoints",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleWaypoints, this, _1));

    slot_.odom = Node::create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleOdometry, this, _1));

    slot_.trajectory = Node::create_subscription<truck_msgs::msg::Trajectory>(
        "/motion/trajectory",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleTrajectory, this, _1));

    signal_.ego = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/ego", rclcpp::QoS(1).reliability(qos));

    signal_.ego_track = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/ego/track", rclcpp::QoS(1).reliability(qos));

    signal_.arc = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/arc",
        rclcpp::QoS(1).reliability(qos));

    signal_.waypoints = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/waypoints",
        rclcpp::QoS(1).reliability(qos));

    signal_.trajectory = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/trajectory",
        rclcpp::QoS(1).reliability(qos));
}

void VisualizationNode::handleOdometry(nav_msgs::msg::Odometry::ConstSharedPtr odom) {
    state_.odom = std::move(odom);
    ++state_.odom_seq_id;

    publishEgo();
    publishEgoTrack();
    publishArc();
}

namespace {

std_msgs::msg::ColorRGBA modeToColor(
    const truck_msgs::msg::ControlMode::ConstSharedPtr& mode) {
    return color::make(*mode);
}

} // namespace

std_msgs::msg::ColorRGBA VisualizationNode::velocityToColor(double velocity, double alpha) const {
    const auto [v_min, v_max] = model_->baseVelocityLimits();
    BOOST_ASSERT(v_min <= 0 && 0 < v_max);

    const double ratio = [&]{
        if (velocity >= 0) {
            return velocity / v_max;
        }

        return abs(v_min) > 0  ? (velocity / v_min) : 0.0;
    }();

    return color::plasma(1 - ratio, alpha);
}

void VisualizationNode::handleTrajectory(truck_msgs::msg::Trajectory::ConstSharedPtr msg) {
    state_.trajectory = std::move(msg);
    publishTrajectory();
}

void VisualizationNode::publishTrajectory() const {
    if (!state_.trajectory) {
        return;
    }

    visualization_msgs::msg::Marker msg;
    msg.header = state_.trajectory->header;
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.frame_locked = true;
    msg.lifetime = params_.ttl;

    msg.scale.x = params_.trajectory_width;
    msg.pose.position.z = params_.trajectory_z_lev;

    msg.points.reserve(state_.trajectory->states.size());
    msg.colors.reserve(state_.trajectory->states.size());

    bool collision = false;
    for (const auto& state: state_.trajectory->states) {
        collision |= state.collision;

        const auto color = collision
            ? color::gray(0.5)
            : velocityToColor(state.velocity, 0.5);

        msg.points.push_back(state.pose.position);
        msg.colors.push_back(color);
    }

    signal_.trajectory->publish(msg);
}

void VisualizationNode::handleMode(truck_msgs::msg::ControlMode::ConstSharedPtr msg) {
    state_.mode = std::move(msg);
    publishEgo();
}

void VisualizationNode::publishEgo() const {
    if (!state_.odom) {
        return;
    }

    visualization_msgs::msg::Marker msg;
    msg.header = state_.odom->header;
    msg.type = visualization_msgs::msg::Marker::CUBE;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.frame_locked = true;
    // always keep last ego marker

    msg.scale.x = model_->shape().length;
    msg.scale.y = model_->shape().width;
    msg.scale.z = params_.ego_height;
    msg.pose = state_.odom->pose.pose;
    msg.pose.position.z = params_.ego_z_lev;
    msg.color = modeToColor(state_.mode);

    signal_.ego->publish(msg);
}

void VisualizationNode::publishEgoTrack() const {
    if (!state_.odom) {
        return;
    }

    if (state_.odom_seq_id % params_.ego_track_rate != 0) {
        return;
    }

    visualization_msgs::msg::Marker msg;
    msg.id = state_.odom_seq_id;
    msg.header = state_.odom->header;
    msg.type = visualization_msgs::msg::Marker::SPHERE;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.frame_locked = true;
    msg.lifetime = params_.ego_track_ttl;

    msg.scale.x = params_.ego_track_width;
    msg.scale.y = params_.ego_track_width;
    msg.scale.z = params_.ego_track_height;
    msg.pose = state_.odom->pose.pose;
    msg.pose.position.z = params_.ego_z_lev;
    msg.color = modeToColor(state_.mode);

    signal_.ego_track->publish(msg);
}

namespace {

geom::Poses arcTrace(const geom::Pose& pose, double curvature, double length) {
    constexpr double step = 0.1;
    constexpr double eps = 1e-2;

    const double abs_curvature = std::abs(curvature);
    if (abs_curvature < eps) {
        return geom::Segment(pose.pos, pose.pos + pose.dir * length).trace(step);
    }

    const geom::Vec2 center = pose.pos + pose.dir.left() / curvature;
    const geom::Vec2 begin = (pose.pos - center).unit();
    const auto delta = clamp(geom::Angle(length * curvature), -geom::PI, geom::PI);

    return geom::Arc(center, abs(1/curvature), begin, delta).trace(step);
}

}  // namespace

void VisualizationNode::publishArc() const {
    if (!state_.odom || !state_.control) {
        return;
    }

    std_msgs::msg::Header header;
    header.frame_id = state_.odom->header.frame_id;
    header.stamp = state_.control->header.stamp;

    constexpr double eps = 1e-3;
    if (std::abs(state_.control->velocity) < eps) {
        visualization_msgs::msg::Marker msg;
        msg.header = header;
        msg.action = visualization_msgs::msg::Marker::DELETE;
        signal_.arc->publish(msg);
        return;
    }

    const auto trace = arcTrace(
        geom::toPose(*state_.odom),
        state_.control->curvature,
        params_.arc_length);

    visualization_msgs::msg::Marker msg;

    msg.header = header;
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.frame_locked = true;
    msg.lifetime = params_.ttl;

    msg.scale.x = params_.arc_width;
    msg.pose.position.z = params_.arc_z_lev;

    msg.points.reserve(trace.size());
    msg.color = color::white(0.6);

    for (const auto& pose: trace) {
        msg.points.push_back(geom::msg::toPoint(pose.pos));
    }

    signal_.arc->publish(msg);
}

void VisualizationNode::handleControl(truck_msgs::msg::Control::ConstSharedPtr control) {
    if (control->header.frame_id != "base") {
        RCLCPP_WARN(
            get_logger(),
            "Expected 'base' frame for cotrol, but got %s. Ignore message!",
            state_.control->header.frame_id.c_str());
        return;
    }

    // publish control only for odometry update
    state_.control = std::move(control);
}

void VisualizationNode::publishWaypoints() const {
    if (!state_.waypoints) {
        return;
    }

    const double size = 2 * params_.waypoints_radius;

    visualization_msgs::msg::Marker marker;

    marker.header = state_.waypoints->header;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.frame_locked = true;
    marker.lifetime = params_.ttl;

    marker.points.resize(state_.waypoints->waypoints.size());
    for (size_t i = 0; i < state_.waypoints->waypoints.size(); ++i) {
        const auto& waypoint = state_.waypoints->waypoints[i];

        marker.scale.x = size;
        marker.scale.y = size;
        marker.scale.z = size;
        marker.color = color::red();

        marker.points[i] = waypoint;
        marker.points[i].z = params_.waypoints_z_lev;
    }

    signal_.waypoints->publish(marker);
}

void VisualizationNode::handleWaypoints(truck_msgs::msg::Waypoints::ConstSharedPtr msg) {
    state_.waypoints = std::move(msg);
    publishWaypoints();
}

}  // namespace truck::visualization