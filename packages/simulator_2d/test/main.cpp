#include <iomanip>
#include <vector>

#include <gtest/gtest.h>

#include "simulator_2d/simulator_engine.h"

using namespace truck::simulator;

void processTestCase(const std::vector<std::pair<int, int>>& script, double update_period) {
    auto model = std::make_unique<truck::model::Model>(
        "/truck/packages/model/config/model.yaml");
    auto engine = SimulatorEngine(std::move(model));
    for (const auto step : script) {
        engine.setBaseControl(step.first, 0);
        for (int j = 0; j < step.second; ++j) {
            engine.advance(update_period);
            const auto truck_state = engine.getTruckState();
            std::cerr << std::fixed << std::setprecision(5)
                << truck_state.getTime().seconds() << ' '
                << truck_state.getBaseOdomPose().pos.x << ' '
                << truck_state.getBaseOdomTwist().velocity << '\n';
        }
    }
}

TEST(SimulatorEngine, straight) {
    std::vector<std::pair<int, int>> script 
        {{10, 500}};
    const double update_period = 0.01;
    
    processTestCase(script, update_period);
}

TEST(SimulatorEngine, straightBackward) {
    std::vector<std::pair<int, int>> script 
        {{10, 100}, {-10, 200}, {10, 300}, {0, 200}};
    const double update_period = 0.01;

    processTestCase(script, update_period);
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
