#include "simulator_2d/simulator_engine.h"

#include "geom/pose.h"
#include "geom/vector.h"
#include "truck_msgs/msg/control.hpp"
#include "truck_msgs/msg/hardware_telemetry.hpp"
#include "truck_msgs/msg/simulation_state.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <rclcpp/rclcpp.hpp>

#include <chrono>

namespace truck::simulator {

class SimulatorNode : public rclcpp::Node {
  public:
    SimulatorNode();

  private:
    void initializeParameters();
    void initializeTopicHandlers();
    void initializeCache(const std::unique_ptr<model::Model>& model);
    void initializeEngine();

    void handleControl(const truck_msgs::msg::Control::ConstSharedPtr control);

    void publishTime(const TruckState& truck_state);
    void publishSimulatorLocalizationMessage(const TruckState& truck_state);
    void publishHardwareOdometryMessage(const TruckState& truck_state);
    void publishTelemetryMessage(const TruckState& truck_state);
    void publishSimulationStateMessage(const TruckState& truck_state);
    void publishLaserScanMessage(const TruckState& truck_state);
    void publishImuMessage(const TruckState& truck_state);
    void publishSimulationState();

    void makeSimulationTick();

    std::optional<tf2::Transform> getLatestTranform(
        const std::string& source, const std::string& target);

    std::unique_ptr<SimulatorEngine> engine_ = nullptr;

    rclcpp::TimerBase::SharedPtr timer_ = nullptr;

    struct Parameters {
        double update_period;
        NoiseGeneratorParams noise_generator;

        struct InitialState {
            double x;
            double y;
            double yaw;
        } init_state;
    } params_;

    struct Cache {
        struct LidarConfig {
            float angle_min;
            float angle_max;
            float angle_increment;
            float range_min;
            float range_max;
        } lidar_config;
    } cache_;

    struct Transforms {
        std::optional<tf2::Transform> ekf_base = std::nullopt;
    } transforms_;

    struct Slots {
        rclcpp::Subscription<truck_msgs::msg::Control>::SharedPtr control = nullptr;
    } slots_;

    struct Signals {
        rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr time = nullptr;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localization = nullptr;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr hardware_odometry = nullptr;
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_publisher = nullptr;
        rclcpp::Publisher<truck_msgs::msg::HardwareTelemetry>::SharedPtr telemetry = nullptr;
        rclcpp::Publisher<truck_msgs::msg::SimulationState>::SharedPtr state = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan = nullptr;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu = nullptr;
    } signals_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;
};

}  // namespace truck::simulator
