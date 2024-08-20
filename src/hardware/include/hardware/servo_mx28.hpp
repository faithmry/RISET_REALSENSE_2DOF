#ifndef SERVO_MX28_HPP
#define SERVO_MX28_HPP

#define ENABLE 1
#define DISABLE 0

//--Dynamixel Address
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

//--Dynamixel Properties
#define PROTOCOL_VERSION 2.0 // See which protocol version is being used in the Dynamixel
#define BAUDRATE 1000000
#define DEVICENAME "/dev/ttyUSB0"   // Check which port is being used on your controller
#define MINIMUM_POSITION_LIMIT 0    // Refer to the Minimum Position Limit of product eManual
#define MAXIMUM_POSITION_LIMIT 4095 // Refer to the Maximum Position Limit of product eManual
#define MOVING_STATUS_THRESHOLD 10  // Resolution of 1 degree is estimated to be 11,375

//--Dynamixel ID
#define ID_YAW 1
#define ID_PITCH 2

//--Servo Limitation
#define YAW_MINIMUM 1137.5
#define YAW_MAXIMUM 2957.5
#define PITCH_MINIMUM 2047.5
#define PITCH_MAXIMUM 2730

//--ROS Header
#include <ros/ros.h>
#include <ros/package.h>

//--Dynamixel Header
#include <hardware/dynamixel_sdk.h>

//--Timer
ros::Timer servo_routine;

//--Subscriber

//--Publisher

//--Prototypes
void servo_routine_callback(const ros::TimerEvent &event);

//--Global Variables

class _ServoMX28
{
private:
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    // Private constructor to prevent instantiation
    _ServoMX28() : portHandler(nullptr), packetHandler(nullptr), index(0), dxl_comm_result{0, 0}, dxl_error{0, 0}, dxl_present_position{0, 0}, dxl_goal_position{0, 0} {}

    // Delete copy constructor and assignment operator
    _ServoMX28(const _ServoMX28 &) = delete;
    _ServoMX28 &operator=(const _ServoMX28 &) = delete;

public:
    int index;
    int dxl_comm_result[2];          // Communication result
    uint8_t dxl_error[2];            // DYNAMIXEL error
    int32_t dxl_present_position[2]; // Read 4 byte Position data
    uint16_t dxl_goal_position[2][2];

    // Static method to get the singleton instance
    static _ServoMX28 &getInstance()
    {
        static _ServoMX28 instance;
        return instance;
    }
    void servo_routine_callback(const ros::TimerEvent &event);
    void servo_mx28_init();
    int set_torque(uint8_t _id, uint8_t _flag);
    int disable_torque(uint8_t _id, uint8_t _flag);
};

#endif