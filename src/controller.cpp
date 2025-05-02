#include "controller.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "utils.hpp"

namespace netizen_robotics
{
    hardware_interface::CallbackReturn MC2408Controller::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        port = (info_.hardware_parameters["serial_port"]);
        RCLCPP_INFO(rclcpp::get_logger("MC2408Controller"), "Hardware parameter - port: %s", port.c_str());
        rclcpp::on_shutdown(std::bind(&MC2408Controller::stop, this));
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MC2408Controller::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MC2408Controller"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MC2408Controller::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        mc2408Handler = new MC2408Handler();
        mc2408Handler->init(port);
        RCLCPP_INFO(rclcpp::get_logger("MC2408Controller"), "Successfully configured!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MC2408Controller::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("MC2408Controller"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> MC2408Controller::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            RCLCPP_INFO(rclcpp::get_logger("MC2408Controller"), "Joint_%d, %s",
                        i, info_.joints[i].name.c_str());
            if (info_.joints[i].name.find("left_wheel_joint") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_left));
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_left));
            }
            if (info_.joints[i].name.find("right_wheel_joint") != std::string::npos)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_right));
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_right));
            }
        }

        for (uint i = 0; i < info_.sensors.size(); i++)
        {
            if (info_.sensors[i].name.find("battery") != std::string::npos)
            {
                for (uint j = 0; j < info_.sensors[i].state_interfaces.size(); j++)
                {
                    RCLCPP_INFO(rclcpp::get_logger("MC2408Controller"), "Sensor_%d, %s | State Interface_%d: %s",
                                i, info_.sensors[i].name.c_str(),
                                j, info_.sensors[i].state_interfaces[j].name.c_str());
                    if (info_.sensors[i].state_interfaces[j].name.find("current") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_current));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("charge") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_charge));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("capacity") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_capacity));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("percentage") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_percentage));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("power_supply_health") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_power_supply_health));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("power_supply_status") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_power_supply_status));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("present") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_present));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("cell_voltage_1") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_cell_voltage_1));
                    }
                    else if (info_.sensors[i].state_interfaces[j].name.find("cell_voltage_2") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_cell_voltage_2));
                    }
                    else if (info_.sensors[i].state_interfaces[j].name.find("cell_voltage_3") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_cell_voltage_3));
                    }
                    else if (info_.sensors[i].state_interfaces[j].name.find("voltage") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_voltage));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("cell_temperature_1") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_cell_temperature_1));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("cell_temperature_2") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_cell_temperature_2));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("cell_temperature_3") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_battery_states_cell_temperature_3));
                    }
                }
            }
            if (info_.sensors[i].name.find("imu_sensor") != std::string::npos)
            {
                for (uint j = 0; j < info_.sensors[i].state_interfaces.size(); j++)
                {
                    RCLCPP_INFO(rclcpp::get_logger("MC2408Controller"), "Sensor_%d, %s | State Interface_%d: %s",
                                i, info_.sensors[i].name.c_str(),
                                j, info_.sensors[i].state_interfaces[j].name.c_str());
                    if (info_.sensors[i].state_interfaces[j].name.find("linear_acceleration.x") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_sensor_states_accel_x));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("linear_acceleration.y") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_sensor_states_accel_y));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("linear_acceleration.z") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_sensor_states_accel_z));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("angular_velocity.x") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_sensor_states_gyro_x));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("angular_velocity.y") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_sensor_states_gyro_y));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("angular_velocity.z") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_sensor_states_gyro_z));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("orientation.x") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_sensor_states_orientation_x));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("orientation.y") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_sensor_states_orientation_y));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("orientation.z") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_sensor_states_orientation_z));
                    }
                    if (info_.sensors[i].state_interfaces[j].name.find("orientation.w") != std::string::npos)
                    {
                        state_interfaces.emplace_back(hardware_interface::StateInterface(
                            info_.sensors[i].name, info_.sensors[i].state_interfaces[j].name, &hw_sensor_states_orientation_w));
                    }
                }
            }
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MC2408Controller::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (auto i = 0u; i < info_.joints.size(); i++)
        {
            if (info_.joints[i].name.find("left") != std::string::npos)
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_left));
            if (info_.joints[i].name.find("right") != std::string::npos)
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_right));
        }
        return command_interfaces;
    }

    hardware_interface::return_type MC2408Controller::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        static bool initialized = false;
        static WheelVelocity last_wheel_velocity;
        static IMUData last_imu;
        static double lastPercentage = 0.0;
        try
        {
            // IMU data
            IMUData imu = mc2408Handler->getIMUData();
            hw_sensor_states_accel_x = imu.accelerometer.x;
            hw_sensor_states_accel_y = imu.accelerometer.y;
            hw_sensor_states_accel_z = imu.accelerometer.z;
            hw_sensor_states_gyro_x = imu.gyroscope.x * M_PI / 180;
            hw_sensor_states_gyro_y = imu.gyroscope.y * M_PI / 180;
            hw_sensor_states_gyro_z = imu.gyroscope.z * M_PI / 180;
            if (initialized)
            {
                const float alpha = 0.98; // Complementary filter coefficient (tuning parameter)
                static float roll = 0.0;
                static float pitch = 0.0;
                static float yaw = 0.0;
                float accel_pitch = atan2(-imu.accelerometer.x, sqrt(imu.accelerometer.y * imu.accelerometer.y + imu.accelerometer.z * imu.accelerometer.z));
                float accel_roll = atan2(imu.accelerometer.y, imu.accelerometer.z);
                float dt = (imu.tick - last_imu.tick) * 0.001;

                float gyro_pitch = pitch + imu.gyroscope.x * dt;
                float gyro_roll = roll + imu.gyroscope.y * dt;
                float gyro_yaw = yaw + imu.gyroscope.z * dt;
                roll = alpha * gyro_roll + (1.0 - alpha) * accel_roll;
                pitch = alpha * gyro_pitch + (1.0 - alpha) * accel_pitch;
                yaw = gyro_yaw;
                tf2::Quaternion q;
                q.setRPY(roll, pitch, yaw);
                q.normalize();
                hw_sensor_states_orientation_x = q.x();
                hw_sensor_states_orientation_y = q.y();
                hw_sensor_states_orientation_z = q.z();
                hw_sensor_states_orientation_w = q.w();
            }
            last_imu = imu;

            // Battery information
            PowerData battery = mc2408Handler->getPowerData();
            hw_battery_states_voltage = battery.batteryVoltage;
            double percentage = battery.batteryVoltage > 12.6 ? 100 : battery.batteryVoltage < 9.0 ? 0.0
                                                                                                   : 100 * (battery.batteryVoltage - 9.0) / (12.6 - 9.0);
            hw_battery_states_percentage = movingAverage(percentage, lastPercentage, 20);
            lastPercentage = hw_battery_states_percentage;

            // Values not measured for the current setup
            hw_battery_states_current = std::numeric_limits<double>::quiet_NaN();
            hw_battery_states_charge = std::numeric_limits<double>::quiet_NaN();
            hw_battery_states_capacity = std::numeric_limits<double>::quiet_NaN();
            hw_battery_states_present = 1.0;
            hw_battery_states_power_supply_health = 0;
            hw_battery_states_power_supply_status = 2;
            hw_battery_states_cell_voltage_1 = std::numeric_limits<double>::quiet_NaN();
            hw_battery_states_cell_voltage_2 = std::numeric_limits<double>::quiet_NaN();
            hw_battery_states_cell_voltage_3 = std::numeric_limits<double>::quiet_NaN();
            hw_battery_states_cell_temperature_1 = std::numeric_limits<double>::quiet_NaN();
            hw_battery_states_cell_temperature_2 = std::numeric_limits<double>::quiet_NaN();
            hw_battery_states_cell_temperature_3 = std::numeric_limits<double>::quiet_NaN();

            // Wheel velocity
            WheelVelocity wheel = mc2408Handler->getWheelVelocity();
            hw_velocities_left = wheel.left;
            hw_velocities_right = wheel.right;
            printf("[Tick: %d] left: %f, right: %f\n", wheel.tick, wheel.left, wheel.right);
            if (initialized)
            {
                // Calculate the position of the wheels using the trapezoidal rule
                float dt = (wheel.tick - last_wheel_velocity.tick) * 0.001;
                hw_positions_left = hw_positions_left + dt * (last_wheel_velocity.left + wheel.left) / 2;
                hw_positions_right = hw_positions_right + dt * (last_wheel_velocity.right + wheel.right) / 2;
            }
            last_wheel_velocity = wheel;
            initialized = true;
            return hardware_interface::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MC2408Controller"), "Error writing to hardware: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

    hardware_interface::return_type MC2408Controller::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        try
        {
            // Left velocity is inverse
            mc2408Handler->setVelocity(hw_commands_left, hw_commands_right);
            return hardware_interface::return_type::OK;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MC2408Controller"), "Error writing to hardware: %s", e.what());
            return hardware_interface::return_type::ERROR;
        }
    }

    void MC2408Controller::stop()
    {
        //@TODO: Update implementation when there is an update to controller manager.
        // Implement workaround to handle ctrl-c termination.
        // Refer to issue - https://github.com/ros-controls/ros2_control/issues/472
        RCLCPP_INFO(rclcpp::get_logger("MC2408Controller"), "ctrl-c called, shutting down...");
        mc2408Handler->stop();
    }

}
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(netizen_robotics::MC2408Controller, hardware_interface::SystemInterface)
