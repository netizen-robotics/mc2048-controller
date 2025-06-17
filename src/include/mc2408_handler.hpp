#ifndef NETIZEN_ROBOTICS_MC2408_HANDLER
#define NETIZEN_ROBOTICS_MC2408_HANDLER

#include <libserial/SerialPort.h>

#include <string>
#include <thread>
#include <mutex>

// DDSM Motor
#define WHEEL_LEFT_SIGN 1
#define WHEEL_RIGHT_SIGN -1

// SAMI-36-3525
// #define WHEEL_LEFT_SIGN -1
// #define WHEEL_RIGHT_SIGN 1

namespace netizen_robotics
{
    enum eResponseCode
    {
        RESPONSE_STATE_IDLE = 0,
        RESPONSE_STATE_ACTIVE,
        RESPONSE_STATE_ACTIVE_VERBOSE,
        RESPONSE_STATE_PID_UPDATED,
        RESPONSE_STATE_EMERGENCY_STOP_ACTIVATED,
        RESPONSE_STATE_EMERGENCY_STOP_DEACTIVATED,
        RESPONSE_STATE_INVALID,
    };

    struct Axis
    {
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
    };

    struct IMUData
    {
        Axis accelerometer;
        Axis gyroscope;
        float temperature = 0.0;
        uint32_t tick = 0;
        uint16_t timeStamp = 0;
    };

    struct PowerData
    {
        uint32_t tick = 0;
        float batteryVoltage = 0.0;
        float currentSense = 0.0;
    };

    struct WheelVelocity
    {
        uint32_t tick = 0;
        float left = 0.0;
        float right = 0.0;
    };

    struct ParsedData
    {
        std::string rawData;
        char type;
        int intData;
        double floatData;
    };

    struct ParsedSerialData
    {
        char command;
        int count = 0;
        std::string rawData;
        ParsedData data[20];
    };

    class MC2408Handler
    {
    public:
        MC2408Handler();
        ~MC2408Handler();
        void setVelocity(float leftSpeed, float rightSpeed);
        bool init(std::string portname);
        void stop();
        void getRTOSInfo();
        PowerData getPowerData();
        IMUData getIMUData();
        WheelVelocity getWheelVelocity();

    private:
        bool running = true;
        std::thread *serial_thread;
        LibSerial::SerialPort port;

        IMUData imuData;
        std::mutex imuDataMutex;

        WheelVelocity wheelVelocity;
        std::mutex wheelVelocityMutex;

        PowerData powerData;
        std::mutex powerMutex;

        char buffer[256];
        int index = 0;

        std::thread *serialThread;
        void handler();
    };

}

#endif