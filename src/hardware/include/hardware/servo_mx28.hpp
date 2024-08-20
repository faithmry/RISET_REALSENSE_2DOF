#ifndef SERVO_MX28_HPP
#define SERVO_MX28_HPP

//--ROS Header
#include <ros/package.h>
#include <ros/ros.h>

//--ROS Messages
#include <auv_msgs/RPY.msg>

//--Dynamixel Header
#include <dynamixel_sdk.h>

//--C++ Headers
#include <stdlib.h>

// Convert degrees to servo position (12-bit resolution)
#define DEG2POSITION *11.375

// Convert servo position to degrees (12-bit resolution)
#define POSITION2DEG *0.08791208791

#define ENABLE 1
#define DISABLE 0

//--Dynamixel Control Table Address for MX-28
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

//--Dynamixel Bytes Length for MX-28
#define LEN_GOAL_POSITION 2
#define LEN_PRESENT_POSITION 2

//--Dynamixel Properties
#define PROTOCOL_VERSION 2.0 // See which protocol version is being used in the Dynamixel
#define BAUDRATE 1000000
#define PATH "/dev/ttyUSB0" // Check which port is being used on your controller

//--Dynamixel ID
#define ID_YAW 1
#define ID_PITCH 2

//--Dynamixel Limit Values
#define MOVING_STATUS_THRESHOLD 10 // Resolution of 1 degree is estimated to be 11,375
#define YAW_MINIMUM 100 DEG2POSITION // Minimum position for yaw servo
#define YAW_MAXIMUM 260 DEG2POSITION // Maximum position for yaw servo
#define PITCH_MINIMUM 180 DEG2POSITION // Minimum position for pitch servo
#define PITCH_MAXIMUM 240 DEG2POSITION // Maximum position for pitch servo

//--Timer
ros::Timer servo_routine;

//--Subscriber
ros::Subscriber rpy_sub;

//--Publisher
ros::Publisher rpy_pub;

class _ServoMX28 {
private:
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
    dynamixel::GroupSyncWrite* groupSyncWrite;
    dynamixel::GroupSyncRead* groupSyncRead;

    int dxl_comm_result; // Communication result
    bool dxl_addparam_result = false; // addParam result
    bool dxl_getdata_result = false; // GetParam result
    uint8_t dxl_error; // DYNAMIXEL error

    // Private constructor to prevent instantiation
    _ServoMX28()
        : portHandler(nullptr)
        , packetHandler(nullptr)
        , groupSyncWrite(nullptr)
        , groupSyncRead(nullptr)
        , dxl_comm_result(0)
        , dxl_addparam_result(false)
        , dxl_getdata_result(false)
        , dxl_error(0)
    {
    }

    // Delete copy constructor and assignment operator
    _ServoMX28(const _ServoMX28&) = delete;
    _ServoMX28& operator=(const _ServoMX28&) = delete;

    uint8_t param_yaw_goal_position[2]; // Goal position parameter written to the yaw servo
    uint8_t param_pitch_goal_position[2]; // Goal position parameter written to the pitch servo
    int16_t yaw_present_position = 0; // Present position
    int16_t pitch_present_position = 0; // Present position

public:
    uint16_t yaw_goal_position = 180 DEG2POSITION; // Goal position
    uint16_t pitch_goal_position = 210 DEG2POSITION; // Goal position

    // Static method to get the singleton instance
    static _ServoMX28& getInstance()
    {
        static _ServoMX28 instance;
        return instance;
    }

    void servo_routine_callback(const ros::TimerEvent& event);
    void rpy_callback(const auv_msgs::RPY::ConstPtr& msg);
    void servo_mx28_init();
    int set_torque(uint8_t _id, uint8_t _flag);
    int disable_torque(uint8_t _id, uint8_t _flag);
};

#endif