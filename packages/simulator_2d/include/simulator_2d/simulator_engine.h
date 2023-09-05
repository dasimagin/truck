#include "model/model.h"
#include "geom/angle.h"
#include "geom/pose.h"
#include "geom/vector.h"

#include <Eigen/Dense>

namespace truck::simulator {

class SimulatorEngine {
  public:
    void start(
        std::unique_ptr<model::Model> &model, const double integration_step = 0.001,
        const double precision = 1e-8);
    void reset();
    geom::Pose getPose() const;
    geom::Angle getSteering() const;
    geom::Vec2 getLinearVelocity() const;
    geom::Vec2 getAngularVelocity() const;
    void setControl(const double velocity, const double acceleration, const double curvature);
    void setControl(const double velocity, const double curvature);
    /**
     * @param time in seconds.
     */
    void advance(const double time = 1.0);

  private:
    Eigen::Matrix<double, 6, 1> calculate_state_delta(
        const Eigen::Matrix<double, 6, 1> &state, const double velocity_delta,
        const double steering_delta);

    struct Parameters {
        double integration_step;
        double precision;
        double steering_velocity;
        double wheelbase;
        double base_to_rear;
        double steering_limit;
    } params_;

    struct Control {
        double velocity = 0.0;
        double acceleration = 0.0;
        double curvature = 0.0;
    } control_;

    enum StateIndex {
        x = 0,
        y = 1,
        rotation = 2,
        steering = 3,
        linear_velocity = 4,
        angular_velocity = 5
    };

    Eigen::Matrix<double, 6, 1> state_;

    std::unique_ptr<model::Model> model_ = nullptr;
};

}  // namespace truck::simulator