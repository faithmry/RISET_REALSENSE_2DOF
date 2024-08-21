#ifndef SERVO_MX28_HPP
#define SERVO_MX28_HPP

//--ROS Header
#include <ros/package.h>
#include <ros/ros.h>

//--ROS Messages
#include <geometry_msgs/Vector3.h>

//--Dynamixel Header
#include <dynamixel_sdk/dynamixel_sdk.h>

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
#define LEN_GOAL_POSITION 4
#define LEN_PRESENT_POSITION 4

//--Dynamixel Properties
#define PROTOCOL_VERSION 2.0 // See which protocol version is being used in the Dynamixel
#define BAUDRATE 1000000
#define PATH "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1:1.0-port0" // Check which port is being used on your controller

//--Dynamixel Offset
#define YAW_OFFSET_POSITION 1024
#define PITCH_OFFSET_POSITION 2559

//--Dynamixel ID
#define YAW_ID 1
#define PITCH_ID 2

class Servo_MX28 {
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
    Servo_MX28()
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
    ~Servo_MX28()
    {
        // Disable DYNAMIXEL Torque
        dxl_comm_result = setTorque(YAW_ID, DISABLE);
        (dxl_comm_result == COMM_SUCCESS) ? printf("Succeeded to disable torque!\n") : printf("Failed to disable torque!\n");
        dxl_comm_result = setTorque(PITCH_ID, DISABLE);
        (dxl_comm_result == COMM_SUCCESS) ? printf("Succeeded to disable torque!\n") : printf("Failed to disable torque!\n");

        // Clear syncwrite parameter storage
        groupSyncWrite->clearParam();
        // Clear syncread parameter storage
        groupSyncRead->clearParam();

        // Delete port handler
        delete portHandler;
        // Delete packet handler
        delete packetHandler;
        // Delete syncwrite instance
        delete groupSyncWrite;
        // Delete syncread instance
        delete groupSyncRead;
    }

    uint8_t param_yaw_goal_position[4]; // Goal position parameter written to the yaw servo
    uint8_t param_pitch_goal_position[4]; // Goal position parameter written to the pitch servo

    std::vector<uint16_t> goal_position = { 2047, 2559 }; // Goal position for yaw and pitch servos
    std::vector<uint16_t> present_position = { 0, 0 }; // Present position for yaw and pitch servos

    int16_t yaw_present_position = 0; // Present position
    int16_t pitch_present_position = 0; // Present position

public:
    // Static method to get the singleton instance
    static Servo_MX28* getInstance()
    {
        static Servo_MX28 instance;
        return &instance;
    }
    Servo_MX28(Servo_MX28 const&) = delete;
    void operator=(Servo_MX28 const&) = delete;

    //--ROS Publisher
    ros::Publisher pub_servo2pc;

    //--ROS Subscriber
    ros::Subscriber sub_pc2servo;

    //--ROS Timer
    ros::Timer tim_routine;

    void init(ros::NodeHandle* nh);
    void initMX28();
    void writeGoalPosition();
    void readPresentPosition();
    int8_t setTorque(uint8_t _id, uint8_t _flag);
    void checkTorque();
    std::vector<uint16_t> jointConvertToPosition(std::vector<double> joint_angle);
    std::vector<double> jointConvertToDegree(std::vector<uint16_t> joint_position);

    void callbackRoutine(const ros::TimerEvent& event);
    void callbackSubscribeRPY(const geometry_msgs::Vector3::ConstPtr& msg);
};

#endif