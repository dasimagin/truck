#pragma once

#include "map/map.h"
#include "navigation/search.h"
#include "navigation/mesh_builder.h"
#include "navigation/graph_builder.h"
#include "truck_msgs/msg/navigation_mesh.hpp"
#include "truck_msgs/msg/navigation_route.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#include <boost/geometry.hpp>
#include "geom/boost/point.h"
#include "geom/transform.h"

namespace truck::routing {

namespace bg = boost::geometry;

using IndexPoint = std::pair<geom::Vec2, size_t>;
using IndexPoints = std::vector<IndexPoint>;
using RTree = bg::index::rtree<IndexPoint, bg::index::rstar<16>>;

struct Route {
    Route();
    Route(const geom::Polyline& polyline);

    double distance(const geom::Vec2& point) const;
    size_t postfixIndex(const geom::Vec2& point, double postfix) const;

    RTree rtree;
    geom::Polyline polyline;
};

struct Cache {
    Cache();
    Cache(const navigation::graph::Graph& graph);

    geom::Polyline findPath(const geom::Vec2& from, const geom::Vec2& to) const;

    RTree rtree;
    navigation::graph::Graph graph;
};

class RoutingNode : public rclcpp::Node {
  public:
    RoutingNode();

  private:
    void initializeParams();
    void initializeTopicHandlers();

    void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void onFinish(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    void updateRoute();

    void makeMeshTick();
    void makeRouteTick();

    void publishMesh() const;
    void publishRoute() const;

    std::optional<geom::Transform> getLatestTranform(
        const std::string& source, const std::string& target) const;

    struct Slots {
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr finish = nullptr;
    } slots_;

    struct Signals {
        rclcpp::Publisher<truck_msgs::msg::NavigationMesh>::SharedPtr mesh = nullptr;
        rclcpp::Publisher<truck_msgs::msg::NavigationRoute>::SharedPtr route = nullptr;
    } signals_;

    struct Timers {
        rclcpp::TimerBase::SharedPtr mesh = nullptr;
        rclcpp::TimerBase::SharedPtr route = nullptr;
    } timers_;

    struct State {
        std::optional<Route> route;
        std::optional<geom::Vec2> ego = std::nullopt;
        std::optional<geom::Vec2> finish = std::nullopt;
    } state_;

    struct Params {
        std::string map_config;

        struct Route {
            double max_ego_dist;
            double postfix_len;
            double spline_step;
        } route;

        navigation::mesh::MeshParams mesh;
        navigation::graph::GraphParams graph;
    } params_;

    Cache cache_;

    std::unique_ptr<map::Map> map_ = nullptr;
    std::unique_ptr<navigation::mesh::MeshBuilder> mesh_builder_ = nullptr;
    std::unique_ptr<navigation::graph::GraphBuilder> graph_builder_ = nullptr;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    const std::string target_frame_ = "world";
};

}  // namespace truck::routing
