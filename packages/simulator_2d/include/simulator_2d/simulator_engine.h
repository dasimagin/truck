#include "model/model.h"

#include "geom/angle.h"
#include "geom/pose.h"
#include "geom/vector.h"

#include <thread>

namespace truck::simulator {

class SimulatorEngine {
    public:
        explicit SimulatorEngine(std::unique_ptr<model::Model> &model) 
            : SimulatorEngine(model, 0.01) {}
        SimulatorEngine(std::unique_ptr<model::Model> &model, double simulation_tick);
        ~SimulatorEngine();
        geom::Vec2 getTruckSizes() const;
        geom::Pose getPose() const;
        geom::Angle getSteering() const;
        void setControl(const double velocity, const double acceleration, const double curvature);

    private:
        void updateState();
        void processSimulation();

        bool isRunning_ = false;

        std::thread running_thread_;

        std::unique_ptr<model::Model> model_ = nullptr;

        struct Parameters {
            double simulation_tick;
        } params_;

        struct State {
            // The coordinates and the yaw.
            geom::Pose pose;
            // The position of the virtual corner in the middle (bicycle model).
            geom::Angle steering;
        } state_;

        struct Control {
            double velocity = 0.0;
            double acceleration = 0.0;
            double curvature = 0.0;
        } control_;
};

} // namespace truck::simulator