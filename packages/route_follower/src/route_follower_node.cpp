#include "route_follower/route_follower_node.h"

#include "geom/msg.h"
#include "model/model.h"
#include "motion/trajectory.h"
#include "speed/greedy_planner.h"

#include <rclcpp/time.hpp>
#include <tf2_ros/qos.hpp>

#include <functional>
#include <optional>

namespace truck::route_follower {

namespace {

Limits<double> velocityLimits(const model::Model& model, double velocity, double time) {
    const auto& vel = model.baseVelocityLimits();

    return {
        std::max(.0, vel.clamp(velocity) - model.baseMaxDeceleration() * time),
        std::min(vel.max, vel.clamp(velocity) + model.baseMaxAcceleration() * time)};
}

template<typename It>
motion::Trajectory makeTrajectory(It begin, It end, const geom::Transform& tf) {
    motion::Trajectory trajectory;
    std::optional<geom::Vec2> prev = std::nullopt;

    for (auto it = begin; it != end; ++it) {
        geom::Vec2 curr = geom::toVec2(*it);

        if (prev.has_value()) {
            trajectory.states.emplace_back();
            trajectory.states.back().pose = tf.apply(
                geom::Pose{.pos = *prev, .dir = geom::AngleVec2::fromVector(curr - *prev)});
        }

        prev = curr;
    }

    trajectory.states.emplace_back();
    trajectory.states.back().pose =
        tf.apply(geom::Pose{.pos = *prev, .dir = trajectory.states.back().pose.dir});

    trajectory.fillDistance();

    return trajectory;
}

}  // namespace

RouteFollowerNode::RouteFollowerNode() : Node("route_follower") {
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));
    slot_.route = this->create_subscription<truck_msgs::msg::NavigationRoute>(
        "/navigation/route",
        rclcpp::QoS(1).reliability(qos),
        std::bind(&RouteFollowerNode::onRoute, this, _1));

    signal_.trajectory =
        this->create_publisher<truck_msgs::msg::Trajectory>("/motion/trajectory", 10);

    params_ = {
        .period = std::chrono::duration<double>(this->declare_parameter("period", 0.1)),
        .safety_margin = this->declare_parameter("safety_margin", 0.3),
    };

    RCLCPP_INFO(this->get_logger(), "period: %.2fs", params_.period.count());
    RCLCPP_INFO(this->get_logger(), "safety_margin: %.2fm", params_.safety_margin);

    speed_params_ = {
        Limits<double>{
            this->declare_parameter("acceleration/min", -0.5),
            this->declare_parameter("acceleration/max", 0.3),
        },
        this->declare_parameter("distance_to_obstacle", 0.7),
    };

    RCLCPP_INFO(
        this->get_logger(), "distance_to_obstacle: %.2fm", speed_params_.distance_to_obstacle);

    RCLCPP_INFO(
        this->get_logger(),
        "acceleration: [%.2f, %.2f]",
        speed_params_.acceleration.min,
        speed_params_.acceleration.max);

    service_.reset = this->create_service<std_srvs::srv::Empty>(
        "/reset_path", std::bind(&RouteFollowerNode::onReset, this, _1, _2));

    slot_.odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        "/ekf/odometry/filtered", 1, std::bind(&RouteFollowerNode::onOdometry, this, _1));

    slot_.grid = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/grid", 1, std::bind(&RouteFollowerNode::onGrid, this, _1));

    using TfCallback = std::function<void(tf2_msgs::msg::TFMessage::SharedPtr)>;

    const TfCallback tf_call = std::bind(&RouteFollowerNode::onTf, this, _1, false);
    const TfCallback static_tf_callback = std::bind(&RouteFollowerNode::onTf, this, _1, true);

    slot_.tf = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", tf2_ros::DynamicListenerQoS(100), tf_call);

    slot_.tf_static = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf_static", tf2_ros::StaticListenerQoS(100), static_tf_callback);

    timer_.main = this->create_wall_timer(
        params_.period, std::bind(&RouteFollowerNode::publishFullState, this));

    signal_.distance_transform =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid/distance_transform", 1);

    model_ = std::make_unique<model::Model>(
        model::load(this->get_logger(), this->declare_parameter("model_config", "")));

    checker_ = std::make_unique<collision::StaticCollisionChecker>(model_->shape());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);
}

void RouteFollowerNode::publishFullState() {
    publishTrajectory();
    publishGridCostMap();
}

void RouteFollowerNode::publishTrajectory() {
    checker_->reset(*state_.distance_transform);

    bool collision = false;
    for (auto& state : state_.trajectory.states) {
        const double margin = checker_->distance(state.pose);
        collision |= margin < params_.safety_margin;

        state.collision = collision;
        state.margin = margin;
    }

    speed::GreedyPlanner(speed_params_, *model_)
        .setScheduledVelocity(state_.scheduled_velocity)
        .fill(state_.trajectory);

    // dirty way to drop invalid trajectories and get error message
    try {
        state_.trajectory.throwIfInvalid(motion::TrajectoryValidations::enableAll(), *model_);
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s", e.what());
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Drop invalid trajectory!");

        state_.trajectory = motion::Trajectory{};
    }

    const double latency = params_.period.count();
    const auto limits = velocityLimits(*model_, state_.localization->velocity, latency);

    if (!state_.trajectory.empty()) {
        const auto scheduled_state = state_.trajectory.byTime(latency);
        state_.scheduled_velocity = limits.clamp(scheduled_state.velocity);
    } else {
        state_.scheduled_velocity = 0.0;
    }

    signal_.trajectory->publish(
        motion::msg::toTrajectory(state_.odometry->header, state_.trajectory));
}

void RouteFollowerNode::publishGridCostMap() {
    if (!state_.distance_transform) {
        return;
    }

    if (!signal_.distance_transform->get_subscription_count()) {
        return;
    }

    constexpr double k_max_distance = 10.0;
    const auto msg = state_.distance_transform->makeCostMap(state_.grid->header, k_max_distance);
    signal_.distance_transform->publish(msg);
}

void RouteFollowerNode::onRoute(const truck_msgs::msg::NavigationRoute::SharedPtr msg) {
    if (!state_.odometry || !state_.distance_transform) {
        return;
    }

    const auto source = std::string{"world"};
    const auto target = state_.odometry->header.frame_id;

    const auto tf_opt = getLatestTranform(source, target);
    if (!tf_opt) {
        RCLCPP_WARN(
            this->get_logger(),
            "Ignore grid, there is no transform from '%s' -> '%s'!",
            source.c_str(),
            target.c_str());
        return;
    }

    state_.trajectory = makeTrajectory(msg->data.begin(), msg->data.end(), *tf_opt);
}

void RouteFollowerNode::onReset(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>) {
    RCLCPP_INFO(this->get_logger(), "Reset path!");
}

void RouteFollowerNode::onOdometry(nav_msgs::msg::Odometry::SharedPtr odometry) {
    state_.localization = geom::toLocalization(*odometry);
    state_.odometry = odometry;
}

void RouteFollowerNode::onGrid(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    if (!state_.odometry) {
        return;
    }

    const auto source = msg->header.frame_id;
    const auto target = state_.odometry->header.frame_id;

    const auto tf_opt = getLatestTranform(source, target);
    if (!tf_opt) {
        RCLCPP_WARN(
            this->get_logger(),
            "Ignore grid, there is no transform from '%s' -> '%s'!",
            source.c_str(),
            target.c_str());
        return;
    }

    // simple hack to apply transform to grid
    msg->header.frame_id = target;
    msg->info.origin = geom::msg::toPose(tf_opt->apply(geom::toPose(msg->info.origin)));

    // distance transfor - cpu intensive operation
    state_.distance_transform = std::make_shared<collision::Map>(
        collision::distanceTransform(collision::Map::fromOccupancyGrid(*msg)));

    state_.grid = msg;
}

void RouteFollowerNode::onTf(tf2_msgs::msg::TFMessage::SharedPtr msg, bool is_static) {
    static const std::string authority;
    for (const auto& transform : msg->transforms) {
        tf_buffer_->setTransform(transform, authority, is_static);
    }
}

std::optional<geom::Transform> RouteFollowerNode::getLatestTranform(
    const std::string& source, const std::string& target) {
    try {
        return geom::toTransform(tf_buffer_->lookupTransform(target, source, rclcpp::Time(0)));
    } catch (const tf2::TransformException& ex) {
        return std::nullopt;
    }
}
}  // namespace truck::route_follower
