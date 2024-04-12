#include "trajectory_planner/state.h"

#include "geom/distance.h"

namespace truck::trajectory_planner {

TruckState::TruckState(const Params& params) : params(params) {}

bool TruckState::IsCollisionFree(const geom::Pose& pose) const noexcept {
    if (!collision_checker) {
        return true;
    }
    return collision_checker->distance(pose) >= params.min_dist_to_obstacle;
}

bool StateArea::IsInside(const State& state) const noexcept {
    return Limits<double>(x_range.min + base_state.pose.pos.x, x_range.max + base_state.pose.pos.x)
               .isMet(state.pose.pos.x) &&
           Limits<double>(y_range.min + base_state.pose.pos.y, y_range.max + base_state.pose.pos.y)
               .isMet(state.pose.pos.y) &&
           Limits<geom::Angle>(
               yaw_range.min + base_state.pose.dir.angle(),
               yaw_range.max + base_state.pose.dir.angle())
               .isMet(state.pose.dir.angle()) &&
           Limits<double>(
               velocity_range.min + base_state.velocity, velocity_range.max + base_state.velocity)
               .isMet(state.velocity);
}

StateSpace& StateSpace::Build(
    const State& ego_state, const StateArea& finish_area, const geom::Polyline& route) noexcept {
    Clear();

    const auto start_pose = ego_state.pose;
    VERIFY(truck_state.IsCollisionFree(start_pose));

    *data = ego_state;

    // TODO - подумать, можно ли быстрее
    auto nearest_segment_it = route.begin();
    auto nearest_segement_distance_sq = geom::distanceSq(
        start_pose.pos, geom::Segment(*nearest_segment_it, *(nearest_segment_it + 1)));
    for (auto it = route.begin(); it + 1 != route.end(); ++it) {
        const auto distance = geom::distanceSq(start_pose.pos, geom::Segment(*it, *(it + 1)));
        if (distance < nearest_segement_distance_sq) {
            nearest_segment_it = it;
            nearest_segement_distance_sq = distance;
        }
    }

    const auto dist_from_milestone = geom::distance(
        *nearest_segment_it,
        geom::projection(
            start_pose.pos, geom::Segment(*nearest_segment_it, *(nearest_segment_it + 1))));

    auto regular_states_end = data + 1;
    auto finish_states_begin = data + params.Size();

    auto longitude_it = geom::UniformStepper(
        &route, params.longitude.Step(), dist_from_milestone, nearest_segment_it);
    longitude_it += params.longitude.limits.min;

    for (int i = 0; i < params.longitude.total_states && longitude_it != route.uend();
         ++i, ++longitude_it) {
        const auto longitude_pose = *longitude_it;
        for (int j = 0; j < params.latitude.total_states; ++j) {
            const auto latitude_pose = geom::Pose(
                longitude_pose.pos + longitude_pose.dir.vec().left() * params.latitude[j],
                longitude_pose.dir);
            Discretization<geom::Angle> forward_yaw = {
                .limits = Limits<geom::Angle>(-geom::PI_2, geom::PI_2),
                .total_states = params.total_forward_yaw_states};
            for (int k = 0; k < forward_yaw.total_states; ++k) {
                for (int v = 0; v < params.velocity.total_states; ++v) {
                    const auto state = State{
                        .pose = geom::Pose(
                            latitude_pose.pos,
                            latitude_pose.dir.angle() + geom::Angle(forward_yaw[k])),
                        .velocity = params.velocity[v]};
                    if (!truck_state.IsCollisionFree(state.pose)) {
                        continue;
                    }
                    if (finish_area.IsInside(state)) {
                        --finish_states_begin;
                        *finish_states_begin = std::move(state);
                    } else {
                        *regular_states_end = std::move(state);
                        ++regular_states_end;
                    }
                }
            }
            Discretization<geom::Angle> backward_yaw = {
                .limits = Limits<geom::Angle>(geom::PI_2, 3 * geom::PI_2),
                .total_states = params.total_backward_yaw_states};
            for (int k = 0; k < backward_yaw.total_states; ++k) {
                for (int v = 0; v < params.velocity.total_states; ++v) {
                    const auto state = State{
                        .pose = geom::Pose(
                            latitude_pose.pos,
                            latitude_pose.dir.angle() + geom::Angle(backward_yaw[k])),
                        .velocity = params.velocity[v]};
                    if (!truck_state.IsCollisionFree(state.pose)) {
                        continue;
                    }
                    if (finish_area.IsInside(state)) {
                        --finish_states_begin;
                        *finish_states_begin = std::move(state);
                    } else {
                        *regular_states_end = std::move(state);
                        ++regular_states_end;
                    }
                }
            }
        }
    }

    start_states = {.data = data, .size = 1};

    finish_states = {
        .data = data + static_cast<int>(finish_states_begin - data),
        .size = static_cast<int>(data + params.Size() - finish_states_begin)};

    regular_states = {.data = data + 1, .size = static_cast<int>(regular_states_end - data - 1)};

    return *this;
}

int StateSpace::Params::Size() const noexcept {
    return longitude.total_states * latitude.total_states *
               (total_forward_yaw_states + total_backward_yaw_states) * velocity.total_states +
           1;
}

int StateSpace::Size() const noexcept {
    return start_states.size + finish_states.size + regular_states.size;
}

StateSpace& StateSpace::Clear() noexcept {
    start_states = {};
    finish_states = {};
    regular_states = {};
    return *this;
}

StateSpace& StateSpace::Reset(State* ptr) noexcept {
    data = ptr;
    return *this;
}

StateSpaceHolder::StateSpaceHolder(
    StateSpace&& state_space, StateSpaceDataPtr&& states_ptr) noexcept
    : state_space(std::move(state_space)), states_ptr(std::move(states_ptr)) {}

StateSpaceHolder MakeStateSpace(const StateSpace::Params& params) {
    auto state_space = StateSpace{.params = params};
    auto states_ptr = std::make_unique<State[]>(state_space.params.Size());
    state_space.Reset(states_ptr.get());
    return {std::move(state_space), std::move(states_ptr)};
}

}  // namespace truck::trajectory_planner
