#include "visualization/visualization_node.h"
#include "visualization/color.h"

#include "common/math.h"
#include "geom/angle.h"
#include "geom/arc.h"
#include "geom/msg.h"
#include "geom/pose.h"
#include "geom/segment.h"

#include <boost/assert.hpp>
#include <tf2_ros/qos.hpp>

#include <array>
#include <functional>
#include <utility>

namespace truck::visualization {

using namespace std::placeholders;

using namespace std::chrono_literals;

using namespace geom::literals;

const double kM2Pi = M_PI * 2;

VisualizationNode::VisualizationNode() : Node("visualization") {
    initializePtrFields();
    initializeParams();
    initializeTopicHandlers();
    initializeCache();
}

void VisualizationNode::initializePtrFields() {
    model_ = model::makeUniquePtr(
        this->get_logger(), Node::declare_parameter<std::string>("model_config", "model.yaml"));

    map_ = std::make_unique<map::Map>(
        map::Map::fromGeoJson(Node::declare_parameter<std::string>("map_config")));
}

void VisualizationNode::initializeParams() {
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

        .mesh_body = this->declare_parameter("mesh.body", ""),
        .mesh_wheel = this->declare_parameter("mesh.wheel", ""),

        .map_z_lev = this->declare_parameter("map.z_lev", 0.0),

        .navigation_mesh_z_lev = this->declare_parameter("navigation_mesh.z_lev", 0.0),
        .navigation_mesh_radius = this->declare_parameter("navigation_mesh.radius", 0.0),

        .navigation_route_z_lev = this->declare_parameter("navigation_route.z_lev", 0.0),
        .navigation_route_width = this->declare_parameter("navigation_route.width", 0.0),
    };
}

void VisualizationNode::initializeTopicHandlers() {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

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

    slot_.telemetry = Node::create_subscription<truck_msgs::msg::HardwareTelemetry>(
        "/hardware/telemetry",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleTelemetry, this, _1));

    slot_.odom = Node::create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleOdometry, this, _1));

    slot_.trajectory = Node::create_subscription<truck_msgs::msg::Trajectory>(
        "/motion/trajectory",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleTrajectory, this, _1));

    slot_.navigation_mesh = Node::create_subscription<truck_msgs::msg::NavigationMesh>(
        "/navigation/mesh",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleNavigationMesh, this, _1));

    slot_.navigation_route = Node::create_subscription<truck_msgs::msg::NavigationRoute>(
        "/navigation/route",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&VisualizationNode::handleNavigationRoute, this, _1));

    signal_.ego = Node::create_publisher<visualization_msgs::msg::MarkerArray>(
        "/visualization/ego", rclcpp::QoS(1).reliability(qos));

    signal_.ego_track = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/ego/track", rclcpp::QoS(1).reliability(qos));

    signal_.arc = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/arc", rclcpp::QoS(1).reliability(qos));

    signal_.waypoints = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/waypoints", rclcpp::QoS(1).reliability(qos));

    signal_.trajectory = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/trajectory", rclcpp::QoS(1).reliability(qos));

    signal_.map = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/map", rclcpp::QoS(1).reliability(qos));

    signal_.navigation_mesh = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/navigation/mesh", rclcpp::QoS(1).reliability(qos));

    signal_.navigation_route = Node::create_publisher<visualization_msgs::msg::Marker>(
        "/visualization/navigation/route", rclcpp::QoS(1).reliability(qos));

    timer_ = Node::create_wall_timer(1s, std::bind(&VisualizationNode::publishMap, this));
}

void VisualizationNode::initializeCacheBodyBaseTf() {
    cache_.body_base_tf = model_->getLatestTranform("base", "body");
}

void VisualizationNode::initializeCacheWheelBaseTfs() {
    for (auto wheel : kAllWheels) {
        cache_.wheel_base_tfs[wheel] = model_->getLatestTranform("base", kWheelFrames[wheel]);
    }
}

void VisualizationNode::initializeCache() {
    initializeCacheBodyBaseTf();
    initializeCacheWheelBaseTfs();
}

void VisualizationNode::handleTelemetry(truck_msgs::msg::HardwareTelemetry::ConstSharedPtr msg) {
    state_.telemetry = msg;
}

void VisualizationNode::handleOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    state_.odom = std::move(msg);
    ++state_.odom_seq_id;

    updateEgo();
    publishEgoTrack();
    publishArc();
}

namespace {

std_msgs::msg::ColorRGBA modeToColor(const truck_msgs::msg::ControlMode::ConstSharedPtr& mode) {
    return color::make(*mode);
}

}  // namespace

std_msgs::msg::ColorRGBA VisualizationNode::velocityToColor(double velocity, double alpha) const {
    const auto [v_min, v_max] = model_->baseVelocityLimits();
    BOOST_ASSERT(v_min <= 0 && 0 < v_max);

    const double ratio = [&] {
        if (velocity >= 0) {
            return velocity / v_max;
        }

        return abs(v_min) > 0 ? (velocity / v_min) : 0.0;
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
    for (const auto& state : state_.trajectory->states) {
        collision |= state.collision;

        const auto color = collision ? color::gray(0.5) : velocityToColor(state.velocity, 0.5);

        msg.points.push_back(state.pose.position);
        msg.colors.push_back(color);
    }

    signal_.trajectory->publish(msg);
}

void VisualizationNode::handleMode(truck_msgs::msg::ControlMode::ConstSharedPtr msg) {
    state_.mode = std::move(msg);
    updateEgo();
}

namespace {

visualization_msgs::msg::Marker makeMeshMarker(
    int id, const std_msgs::msg::Header& header, const std::string& mesh,
    const std_msgs::msg::ColorRGBA& color) {
    visualization_msgs::msg::Marker msg;
    msg.id = id;
    msg.header = header;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.frame_locked = true;
    // always keep last ego marker

    msg.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    msg.mesh_resource = mesh;
    msg.scale.x = 1;
    msg.scale.y = 1;
    msg.scale.z = 1;

    msg.color = color;

    return msg;
}

}  // namespace

void VisualizationNode::updateWheelsSpin() {
    const auto now_seconds = now().seconds();
    const auto before = state_.last_ego_update_seconds.value_or(now_seconds);
    const auto time = now_seconds - before;

    const model::WheelVelocity wheel_velocity = {
        geom::Angle(state_.telemetry->rear_left_wheel_velocity),
        geom::Angle(state_.telemetry->rear_right_wheel_velocity),
        geom::Angle(state_.telemetry->front_left_wheel_velocity),
        geom::Angle(state_.telemetry->front_right_wheel_velocity)};

    for (auto wheel : kAllWheels) {
        const double velocity = [&]() {
            switch (wheel) {
                case WheelIndex::kFrontLeft:
                    return wheel_velocity.front_left.radians();
                case WheelIndex::kFrontRight:
                    return -wheel_velocity.front_right.radians();
                case WheelIndex::kRearLeft:
                    return wheel_velocity.rear_left.radians();
                case WheelIndex::kRearRight:
                default:
                    return -wheel_velocity.rear_right.radians();
            }
        }();

        const double angle = state_.wheel_spin_angles[wheel] + velocity * time;
        state_.wheel_spin_angles[wheel] = std::fmod(angle, kM2Pi);
    }

    state_.last_ego_update_seconds = now_seconds;
}

void VisualizationNode::updateEgo() {
    if (!state_.odom || !state_.mode || !state_.telemetry) {
        return;
    }

    updateWheelsSpin();
    publishEgo();
}

void VisualizationNode::publishEgo() const {
    tf2::Transform base_to_odom;
    tf2::fromMsg(state_.odom->pose.pose, base_to_odom);

    auto color = modeToColor(state_.mode);
    auto body_msg = makeMeshMarker(0, state_.odom->header, params_.mesh_body, color);

    const auto body_tf = base_to_odom * cache_.body_base_tf;
    tf2::toMsg(body_tf, body_msg.pose);

    visualization_msgs::msg::MarkerArray msg_array;
    msg_array.markers.push_back(body_msg);

    for (auto wheel : kAllWheels) {
        const double y_angle = state_.wheel_spin_angles[wheel];
        const double z_angle = [&]() {
            switch (wheel) {
                case WheelIndex::kFrontLeft:
                    return state_.telemetry->current_left_steering;
                case WheelIndex::kFrontRight:
                    return state_.telemetry->current_right_steering;
                default:
                    return 0.0;
            }
        }();

        auto rotation = tf2::Quaternion::getIdentity();
        rotation.setRPY(0, y_angle, z_angle);
        const auto wheel_tf =
            base_to_odom * cache_.wheel_base_tfs[wheel] * tf2::Transform(rotation);

        auto wheel_msg = makeMeshMarker(wheel + 1, state_.odom->header, params_.mesh_wheel, color);
        tf2::toMsg(wheel_tf, wheel_msg.pose);

        msg_array.markers.push_back(wheel_msg);
    }

    signal_.ego->publish(msg_array);
}

void VisualizationNode::publishEgoTrack() const {
    if (!state_.odom || !state_.mode) {
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

    return geom::Arc(center, abs(1 / curvature), begin, delta).trace(step);
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

    const auto trace =
        arcTrace(geom::toPose(*state_.odom), state_.control->curvature, params_.arc_length);

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

    for (const auto& pose : trace) {
        msg.points.push_back(geom::msg::toPoint(pose.pos));
    }

    signal_.arc->publish(msg);
}

void VisualizationNode::handleControl(truck_msgs::msg::Control::ConstSharedPtr msg) {
    if (msg->header.frame_id != "base") {
        RCLCPP_WARN(
            get_logger(),
            "Expected 'base' frame for cotrol, but got %s. Ignore message!",
            state_.control->header.frame_id.c_str());
        return;
    }

    // publish control only for odometry update
    state_.control = std::move(msg);
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

void VisualizationNode::publishMap() const {
    visualization_msgs::msg::Marker msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    // msg.header.frame_id = "odom_ekf";
    msg.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.color = color::gray(0.6);
    msg.pose.position.z = params_.map_z_lev;

    const auto& polygons = map_->polygons();
    VERIFY(polygons.size() == 1);
    const auto& polygon = polygons[0];

    for (const geom::Triangle& triangle : polygon.triangles()) {
        msg.points.push_back(geom::msg::toPoint(triangle.p1));
        msg.points.push_back(geom::msg::toPoint(triangle.p2));
        msg.points.push_back(geom::msg::toPoint(triangle.p3));
    }

    signal_.map->publish(msg);
}

void VisualizationNode::handleNavigationMesh(truck_msgs::msg::NavigationMesh::ConstSharedPtr msg) {
    state_.navigation_mesh = std::move(msg);
    publishNavigationMesh();
}

void VisualizationNode::publishNavigationMesh() const {
    if (!state_.navigation_mesh) {
        return;
    }

    const double size = 2 * params_.navigation_mesh_radius;

    visualization_msgs::msg::Marker msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.color = color::white(0.8);
    msg.scale.x = size;
    msg.scale.y = size;
    msg.scale.z = size;
    msg.pose.position.z = params_.navigation_mesh_z_lev;

    for (const auto& point : state_.navigation_mesh->data) {
        msg.points.emplace_back(point);
    }

    signal_.navigation_mesh->publish(msg);
}

void VisualizationNode::handleNavigationRoute(
    truck_msgs::msg::NavigationRoute::ConstSharedPtr msg) {
    state_.navigation_route = std::move(msg);
    publishNavigationRoute();
}

void VisualizationNode::publishNavigationRoute() const {
    if (!state_.navigation_route) {
        return;
    }

    const size_t postfix_index = state_.navigation_route->postfix_index;
    const size_t points_count = state_.navigation_route->data.size();

    visualization_msgs::msg::Marker msg;
    msg.header.stamp = now();
    msg.header.frame_id = "world";
    msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.color = color::blue(0.5);
    msg.scale.x = params_.navigation_route_width;
    msg.pose.position.z = params_.navigation_route_z_lev;

    for (size_t i = postfix_index; i < points_count; i++) {
        msg.points.emplace_back(state_.navigation_route->data[i]);
    }

    signal_.navigation_route->publish(msg);
}

}  // namespace truck::visualization
