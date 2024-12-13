#ifndef MC2408_CONTROLLER
#define MC2408_CONTROLLER

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "mc2408_handler.hpp"

namespace netizen_robotics
{
    class MC2408Controller : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MC2408Controller)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        void stop();

    private:
        std::string port;
        MC2408Handler *mc2408Handler;

        double hw_commands_left = 0.0;
        double hw_commands_right = 0.0;

        double hw_positions_left = 0.0;
        double hw_positions_right = 0.0;
        double hw_velocities_left = 0.0;
        double hw_velocities_right = 0.0;

        double hw_sensor_states_accel_x = 0.0;
        double hw_sensor_states_accel_y = 0.0;
        double hw_sensor_states_accel_z = 0.0;
        double hw_sensor_states_gyro_x = 0.0;
        double hw_sensor_states_gyro_y = 0.0;
        double hw_sensor_states_gyro_z = 0.0;
        double hw_sensor_states_orientation_x = 0.0;
        double hw_sensor_states_orientation_y = 0.0;
        double hw_sensor_states_orientation_z = 0.0;
        double hw_sensor_states_orientation_w = 0.0;

        double hw_battery_states_voltage = 0.0;
        double hw_battery_states_current = 0.0;
        double hw_battery_states_charge = 0.0;
        double hw_battery_states_capacity = 0.0;
        double hw_battery_states_percentage = 0.0;
        double hw_battery_states_power_supply_status = 0.0;
        double hw_battery_states_power_supply_health = 0.0;
        double hw_battery_states_present = 0.0;
        double hw_battery_states_cell_voltage_1 = 0.0;
        double hw_battery_states_cell_voltage_2 = 0.0;
        double hw_battery_states_cell_voltage_3 = 0.0;
        double hw_battery_states_cell_temperature_1 = 0.0;
        double hw_battery_states_cell_temperature_2 = 0.0;
        double hw_battery_states_cell_temperature_3 = 0.0;
    };

}

#endif