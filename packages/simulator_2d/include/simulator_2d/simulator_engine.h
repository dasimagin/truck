#include "model/model.h"
#include "geom/angle.h"
#include "geom/pose.h"
#include "geom/vector.h"

#include <Eigen/Dense>

namespace truck::simulator {

class SimulatorEngine {
  public:
    SimulatorEngine(const model::Model& model, 
        double integration_step = 0.001, double precision = 1e-8);

    void reset_rear(double x, double y, double yaw,
        double steering, double linear_velocity);
    void reset_rear();
    void reset_base(double x, double y, double yaw,
        double steering, double linear_velocity);

    const rclcpp::Time& getTime() const;
    geom::Pose getBasePose() const;
    model::Steering getCurrentSteering() const;
    model::Steering getTargetSteering() const;
    model::Twist getBaseTwist() const;
    geom::Vec2 getBaseLinearVelocity() const;
    geom::Vec2 getBaseAngularVelocity() const;

    void setBaseControl(double velocity, double acceleration, double curvature);
    void setBaseControl(double velocity, double curvature);
    void advance(double seconds = 1.0);

  private:
    enum StateIndex { 
        x = 0, 
        y = 1, 
        yaw = 2, 
        steering = 3, 
        linear_velocity = 4
    };

    typedef Eigen::Matrix<double, 5, 1> State;

    double getCurrentRearCurvature() const;
    State calculateStateDerivative(const State &state);
    void validateAcceleration(double& target_velocity);
    State calculateRK4();

    struct Parameters {
        double integration_step;
        double precision;    
    } params_;

    struct Cache {
        double integration_step_2;
        double integration_step_6;
        double inverse_integration_step;
        double inverse_wheelbase_length;
        double wheelbase_width_2;
    } cache_;

    struct Control {
        double velocity = 0.0;
        double acceleration = 0.0;
        double curvature = 0.0;
    } control_;

    rclcpp::Time time_;

    State rear_ax_state_;

    const model::Model model_;
};

}  // namespace truck::simulator
