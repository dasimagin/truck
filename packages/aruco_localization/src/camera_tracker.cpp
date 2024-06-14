#include "camera_tracker.hpp"
#include "math_helpers.hpp"

#include <functional>
#include <rclcpp/rclcpp.hpp>

namespace rosaruco {

const static std::string LOGGER_NAME = "CameraTracker";

CameraTracker::CameraTracker(int marker_count) : graph_(marker_count) {
    to_anchor_.resize(marker_count);
}

void CameraTracker::update(
    const std::vector<int>& ids, const std::vector<Transform>& from_marker_to_cam) {
    if (ids.empty()) {
        return;
    }

    if (anchor_id_ == -1) {
        anchor_id_ = ids[0];
    }

    for (size_t i = 0; i < ids.size(); i++) {
        for (size_t j = 0; j < ids.size(); j++) {
            if (i != j) {
                graph_.addTransform(
                    ids[i], ids[j], from_marker_to_cam[j].inverse() * from_marker_to_cam[i]);
            }
        }
    }

    std::vector<Transform> transforms_to_anchor;
    std::vector<double> errors;

    graph_.getBestTransformFromStartNode(anchor_id_, ids, transforms_to_anchor, errors);

    const size_t best_visible_idx = std::min_element(errors.begin(), errors.end()) - errors.begin();

    if (std::isinf(errors[best_visible_idx])) {
        RCLCPP_ERROR(
            rclcpp::get_logger(LOGGER_NAME),
            "Current position can not be calculated: No visible marker reachable from an anchor "
            "marker.");
        return;
    }

    auto from_best_visible_to_anchor =
        transforms_to_anchor[best_visible_idx] * from_marker_to_cam[best_visible_idx].inverse();

    Pose new_pose;
    new_pose.orientation = from_best_visible_to_anchor.getRotation()
                           * tf2::Quaternion(tf2::Vector3(0, 1, 0), -M_PI / 2)
                           * tf2::Quaternion(tf2::Vector3(1, 0, 0), M_PI / 2);
    new_pose.point = from_best_visible_to_anchor({0, 0, 0});

    current_pose_ = new_pose;

    for (size_t i = 0; i < ids.size(); i++) {
        if (!std::isinf(errors[ids[i]])) {
            to_anchor_[ids[i]] = transforms_to_anchor[i];
        } else {
            RCLCPP_WARN(
                rclcpp::get_logger(LOGGER_NAME),
                "Marker with id = %d is not reachable from an anchor marker.",
                ids[i]);
        }
    }
}

const std::optional<Transform>& CameraTracker::getTransformToAnchor(int from_id) const {
    return to_anchor_[from_id];
}

Pose CameraTracker::getPose() { return current_pose_; }

}  // namespace rosaruco
