#pragma once

#include "geom/angle.h"
#include "common/math.h"

#include "yaml-cpp/yaml.h"

#include <boost/assert.hpp>

#include <iostream>

namespace truck::model {

struct WheelBase {
    WheelBase(const YAML::Node& node);

    double width;
    double length;
    double base_to_rear;
};

struct SteeringLimit {
    geom::Angle inner;
    geom::Angle outer;
};

struct VehicleLimits {
    VehicleLimits(const YAML::Node& node);

    double max_abs_curvature;
    SteeringLimit steering;
    Limits<double> velocity;
    Limits<double> acceleration;
};

struct Params {
    Params(const YAML::Node& node);
    Params(const std::string& config_path);

    WheelBase wheel_base;
    model::VehicleLimits limits;
    double wheel_radius;
};

}  // namespace truck::model