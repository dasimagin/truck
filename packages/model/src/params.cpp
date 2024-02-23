#include <model/params.h>

namespace truck::model {

namespace {

SteeringLimit toSteeringLimits(const YAML::Node& node) {
    return {
        geom::Angle::fromDegrees(node["inner"].as<double>()),
        geom::Angle::fromDegrees(node["outer"].as<double>())};
}

template <typename T>
Limits<double> toLimits(const YAML::Node& node) {
    return {node["min"].as<double>(), node["max"].as<double>()};
}

geom::Vec2 toVector(const YAML::Node& node) {
    const auto x = node["x"].as<double>();
    const auto y = node["y"].as<double>();
    return geom::Vec2(x, y);
}

}  // namespace

using namespace geom::literals;

Shape::Shape() {}

Shape::Shape(const YAML::Node& node)
    : width(node["width"].as<double>())
    , length(node["length"].as<double>())
    , base_to_rear(node["base_to_rear"].as<double>())
    , circles_count(node["circles_count"].as<int>()) {
    BOOST_VERIFY(width > 0);
    BOOST_VERIFY(length > 0);
    BOOST_VERIFY(base_to_rear > 0);
    BOOST_VERIFY(length > base_to_rear);
    BOOST_VERIFY(circles_count * 2 * radius() > length);
}

double Shape::radius() const {
    return width / 2;
}

std::vector<geom::Vec2> Shape::getCircleDecomposition(const geom::Pose& ego_pose) const {
    std::vector<geom::Vec2> points;
    const double pos_first = -base_to_rear + radius();
    const double pos_step = (length - 2 * radius()) / (circles_count - 1);

    for (int i = 0; i < circles_count; i++) {
        double offset = pos_first + (i * pos_step);
        points.push_back(ego_pose.pos + offset * ego_pose.dir);
    }

    return points;
}

WheelBase::WheelBase(const YAML::Node& node)
    : width(node["width"].as<double>())
    , length(node["length"].as<double>())
    , base_to_rear(node["base_to_rear"].as<double>()) {
    BOOST_VERIFY(width > 0);
    BOOST_VERIFY(length > 0);
    BOOST_VERIFY(base_to_rear > 0);
    BOOST_VERIFY(length > base_to_rear);
}

VehicleLimits::VehicleLimits(const YAML::Node& node)
    : max_abs_curvature(node["max_abs_curvature"].as<double>())
    , steering_velocity(node["steering_velocity"].as<double>())
    , steering{toSteeringLimits(node["steering"])}
    , velocity{toLimits<double>(node["velocity"])}
    , max_acceleration(node["max_acceleration"].as<double>())
    , max_deceleration(node["max_deceleration"].as<double>()) {
    BOOST_VERIFY(max_abs_curvature >= 0);

    BOOST_VERIFY(0_deg <= steering.inner && steering.inner < 90_deg);
    BOOST_VERIFY(0_deg <= steering.outer && steering.outer < 90_deg);

    BOOST_VERIFY(velocity.min <= 0);
    BOOST_VERIFY(0 < velocity.max);

}

ServoAngles::ServoAngles(const YAML::Node& node)
    : left(geom::Angle::fromDegrees(node["left"].as<double>()))
    , right(geom::Angle::fromDegrees(node["right"].as<double>())) {
    BOOST_VERIFY(0_deg <= left && left < 180_deg);
    BOOST_VERIFY(0_deg <= right && right < 180_deg);
}

Wheel::Wheel(const YAML::Node& node)
    : radius(node["radius"].as<double>())
    , width(node["width"].as<double>()) {
    BOOST_VERIFY(radius > 0);
    BOOST_VERIFY(width > 0);
}

Lidar::Lidar(const YAML::Node& node)
    : from_base(toVector(node["from_base"]))
    , angle_min(geom::Angle::fromDegrees(node["angle_min"].as<double>()))
    , angle_max(geom::Angle::fromDegrees(node["angle_max"].as<double>()))
    , angle_increment(geom::Angle::fromDegrees(node["angle_increment"].as<double>()))
    , range_min(node["range_min"].as<float>())
    , range_max(node["range_max"].as<float>()) {
    BOOST_VERIFY(angle_min.radians() >= 0);
    BOOST_VERIFY(angle_max > angle_min);
    BOOST_VERIFY(angle_increment.radians() > 0);
    BOOST_VERIFY(range_min >= 0);
    BOOST_VERIFY(range_max > range_min);
}

Params::Params(const YAML::Node& node)
    : shape(node["shape"])
    , wheel_base(node["wheel_base"])
    , wheel(node["wheel"])
    , lidar(node["lidar"])
    , limits(node["limits"])
    , gear_ratio(node["gear_ratio"].as<double>())
    , servo_home_angles(node["servo_home_angles"]) {
    BOOST_VERIFY(gear_ratio > 0);
}

Params::Params(const std::string& config_path) : Params(YAML::LoadFile(config_path)) {}

}  // namespace truck::model
