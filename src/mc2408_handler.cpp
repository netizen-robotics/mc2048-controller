
#include <chrono>
#include <cstdio>

#include "mc2408_handler.hpp"
#include "types.hpp"

#include "rclcpp/rclcpp.hpp"

namespace netizen_robotics
{
    MC2408Handler::MC2408Handler()
    {
    }

    bool MC2408Handler::init(std::string portname)
    {
        try
        {
            port.Open(portname);
            port.SetBaudRate(LibSerial::BaudRate::BAUD_230400);
            port.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
            port.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
            port.SetParity(LibSerial::Parity::PARITY_NONE);
            port.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
            RCLCPP_INFO(rclcpp::get_logger("MC2408ControllerHandler"), "Connected to portname %s", portname.c_str());
            port.Write("sa\n");
            port.DrainWriteBuffer();
            serialThread = new std::thread(&MC2408Handler::handler, this);
            return true;
        }
        catch (const LibSerial::OpenFailed &)
        {
            RCLCPP_ERROR(rclcpp::get_logger("MC2408ControllerHandler"), "Failed to connect to %s", portname.c_str());
            return false;
        }
    }

    void MC2408Handler::handler()
    {
        running = true;
        size_t ms_timeout = 100;
        char data_byte;

        auto start = std::chrono::high_resolution_clock::now();

        while (running)
        {
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            if (duration.count() > 500)
            {
                port.Write("t\n");
                port.DrainWriteBuffer();
                start = end;
            }
            while (port.IsDataAvailable())
            {
                try
                {
                    port.ReadByte(data_byte, ms_timeout);
                    if (data_byte == '\n')
                    {
                        buffer[index] = '\0';
                        index = 0;
                        switch (buffer[0])
                        {
                        case '#':
                            RCLCPP_INFO(rclcpp::get_logger("MC2408ControllerHandler"), "%s", &buffer[1]);
                            break;
                        case '*':
                        {
                            uint8_t state = parseHexStringToUint8(buffer + 1);
                            switch (state)
                            {
                            case RESPONSE_STATE_IDLE:
                                RCLCPP_INFO(rclcpp::get_logger("MC2408ControllerHandler"), "Controller Idle");
                                break;
                            case RESPONSE_STATE_ACTIVE:
                                RCLCPP_INFO(rclcpp::get_logger("MC2408ControllerHandler"), "Controller Idle");
                                break;
                            case RESPONSE_STATE_ACTIVE_VERBOSE:
                                RCLCPP_INFO(rclcpp::get_logger("MC2408ControllerHandler"), "Controller Active");
                                break;
                            case RESPONSE_STATE_EMERGENCY_STOP_ACTIVATED:
                                RCLCPP_INFO(rclcpp::get_logger("MC2408ControllerHandler"), "Emergency stop activated");
                                break;
                            case RESPONSE_STATE_EMERGENCY_STOP_DEACTIVATED:
                                RCLCPP_INFO(rclcpp::get_logger("MC2408ControllerHandler"), "Emergency stop deactivated");
                                break;
                            case RESPONSE_STATE_INVALID:
                                RCLCPP_INFO(rclcpp::get_logger("MC2408ControllerHandler"), "Controller under invalid state for command");
                                break;
                            }
                            break;
                        }
                        case '!':
                            RCLCPP_INFO(rclcpp::get_logger("MC2408ControllerHandler"), "Invalid Command: %s", &buffer[1]);
                            break;
                        case 'P':
                            powerMutex.lock();
                            powerData.tick = parseHexStringToUint32(buffer + 1);
                            powerData.batteryVoltage = parseHexStringToFloat(buffer + 10);
                            powerData.currentSense = parseHexStringToFloat(buffer + 19);
                            powerMutex.unlock();
                            break;
                        case 'D':
                            if (buffer[1] == 'V')
                            {
                            }
                            else
                            {
                                wheelVelocityMutex.lock();
                                wheelVelocity.tick = parseHexStringToUint32(buffer + 1);
                                wheelVelocity.left = WHEEL_LEFT_SIGN * parseHexStringToFloat(buffer + 10);
                                wheelVelocity.right = WHEEL_RIGHT_SIGN * parseHexStringToFloat(buffer + 19);
                                wheelVelocityMutex.unlock();
                            }
                            break;
                        case 'I':
                            imuDataMutex.lock();
                            imuData.tick = parseHexStringToUint32(buffer + 1);
                            imuData.timeStamp = parseHexStringToUint32(buffer + 10);
                            imuData.temperature = parseHexStringToFloat(buffer + 15);
                            imuData.accelerometer.x = parseHexStringToFloat(buffer + 24);
                            imuData.accelerometer.y = parseHexStringToFloat(buffer + 33);
                            imuData.accelerometer.z = parseHexStringToFloat(buffer + 42);
                            imuData.gyroscope.x = parseHexStringToFloat(buffer + 51);
                            imuData.gyroscope.y = parseHexStringToFloat(buffer + 60);
                            imuData.gyroscope.z = parseHexStringToFloat(buffer + 69);
                            imuDataMutex.unlock();
                            break;
                        default:
                            break;
                        }
                    }
                    else
                    {
                        buffer[index++] = data_byte;
                    }
                }
                catch (const LibSerial::ReadTimeout &)
                {
                    RCLCPP_ERROR(rclcpp::get_logger("MC2408ControllerHandler"), "Port read timeout");
                    running = false;
                }
            }
        }
    }

    PowerData MC2408Handler::getPowerData()
    {
        powerMutex.lock();
        PowerData _powerData = powerData;
        powerMutex.unlock();
        return _powerData;
    }

    IMUData MC2408Handler::getIMUData()
    {
        imuDataMutex.lock();
        IMUData _imuData = imuData;
        imuDataMutex.unlock();
        return _imuData;
    }

    WheelVelocity MC2408Handler::getWheelVelocity()
    {
        wheelVelocityMutex.lock();
        WheelVelocity _wheelVelocity = wheelVelocity;
        wheelVelocityMutex.unlock();
        return _wheelVelocity;
    }

    void MC2408Handler::getRTOSInfo()
    {
        port.Write("rf\n");
        port.DrainWriteBuffer();
    }

    void MC2408Handler::setVelocity(float leftSpeed, float rightSpeed)
    {
        char buffer[20];
        sprintf(buffer, "m%08x|%08x\n", float2byte(WHEEL_LEFT_SIGN * leftSpeed), float2byte(WHEEL_RIGHT_SIGN * rightSpeed));
        port.Write(buffer);
        port.DrainWriteBuffer();
    }

    void MC2408Handler::stop()
    {
        port.Write("si\n");
        port.DrainWriteBuffer();
        port.Close();
        running = false;
        serialThread->join();
        RCLCPP_INFO(rclcpp::get_logger("MC2408ControllerHandler"), "Robot stopped");
    }

    MC2408Handler::~MC2408Handler()
    {
        RCLCPP_INFO(rclcpp::get_logger("MC2408ControllerHandler"), "Stopping handler");
        stop();
    }
}