#include "hardware/servo_mx28.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_mx28");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(0);

    //--Initialize Servo
    _ServoMX28::getInstance().servo_mx28_init();

    //--Subscriber
    rpy_sub = nh.subscribe("/master/servo_goal_pose", 1, &_ServoMX28::rpy_callback, &_ServoMX28::getInstance());

    //--Publisher
    rpy_pub = nh.advertise<geometry_msgs::Vector3>("/hardware/servo_current_pose", 1);

    //--Timer
    servo_routine = nh.createTimer(ros::Duration(0.1), &_ServoMX28::servo_routine_callback, &_ServoMX28::getInstance());
    spinner.spin();
    return 0;
}

void _ServoMX28::rpy_callback(const geometry_msgs::Vector3::ConstPtr &msg)
{
    yaw_goal_position = msg->z DEG2POSITION;
    pitch_goal_position = msg->y DEG2POSITION;

    printf("Yaw: %d, Pitch: %d\n", yaw_goal_position, pitch_goal_position);
}

void _ServoMX28::servo_routine_callback(const ros::TimerEvent &event)
{
    static int8_t yaw_direction = -1;  // 1: counterclockwise, -1: clockwise
    static int8_t pitch_direction = 1; // 1: counterclockwise, -1: clockwise
    // Check if it's time to change the direction
    if (yaw_goal_position > 4000 | yaw_goal_position < 0)
    {
        yaw_direction *= -1; // Change the direction
    }

    if (pitch_goal_position > 4000 | pitch_goal_position < 0)
    {
        pitch_direction *= -1; // Change the direction
    }

    // Calculate the new goal position
    yaw_goal_position += (100 * yaw_direction);
    pitch_goal_position += (100 * pitch_direction);

    if (abs(yaw_goal_position - yaw_present_position) > 5 || abs(pitch_goal_position - pitch_present_position) > 5)
    {
        memcpy(param_yaw_goal_position, &yaw_goal_position, sizeof(yaw_goal_position));
        memcpy(param_pitch_goal_position, &pitch_goal_position, sizeof(yaw_goal_position));

        dxl_addparam_result = groupSyncWrite->addParam(PITCH_ID, param_pitch_goal_position);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", PITCH_ID);
            return;
        }

        // Add Dynamixel goal position value to the Syncwrite storage
        dxl_addparam_result = groupSyncWrite->addParam(YAW_ID, param_yaw_goal_position);
        if (dxl_addparam_result != true)
        {
            fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", YAW_ID);
            return;
        }

        // Syncwrite goal position
        dxl_comm_result = groupSyncWrite->txPacket();
        if (dxl_comm_result != COMM_SUCCESS)
        {
            fprintf(stderr, "%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        }

        // Clear syncwrite parameter storage
        groupSyncWrite->clearParam();
    }
    // Syncread present position
    dxl_comm_result = groupSyncRead->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        fprintf(stderr, "%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }

    // Check if groupSyncRead data of Dynamixel present position is available
    dxl_getdata_result = groupSyncRead->isAvailable(YAW_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    if (dxl_getdata_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", YAW_ID);
        return;
    }

    dxl_getdata_result = groupSyncRead->isAvailable(PITCH_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    if (dxl_getdata_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", PITCH_ID);
        return;
    }

    // Get present position value
    yaw_present_position = groupSyncRead->getData(YAW_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    pitch_present_position = groupSyncRead->getData(PITCH_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

    geometry_msgs::Vector3 rpy_msg;
    rpy_msg.z = yaw_present_position POSITION2DEG;
    rpy_msg.y = pitch_present_position POSITION2DEG;

    if (rpy_msg.z > 180)
        rpy_msg.z -= 360;
    else if (rpy_msg.z < -180)
        rpy_msg.z += 360;

    if (rpy_msg.y > 180)
        rpy_msg.y -= 360;
    else if (rpy_msg.y < -180)
        rpy_msg.y += 360;

    rpy_pub.publish(rpy_msg);
}

void _ServoMX28::servo_mx28_init()
{
    portHandler = dynamixel::PortHandler::getPortHandler(PATH);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    groupSyncRead = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

    // Open port
    (portHandler->openPort()) ? printf("Succeeded to open the port!\n") : printf("Failed to open the port!\n");

    // Set port baudrate
    (portHandler->setBaudRate(BAUDRATE)) ? printf("Succeeded to change the baudrate!\n") : printf("Failed to change the baudrate!\n");

    // Enable DYNAMIXEL Torque
    dxl_comm_result = set_torque(YAW_ID, ENABLE);
    (dxl_comm_result == COMM_SUCCESS) ? printf("Succeeded to enable torque!\n") : printf("Failed to enable torque!\n");
    dxl_comm_result = set_torque(PITCH_ID, ENABLE);
    (dxl_comm_result == COMM_SUCCESS) ? printf("Succeeded to enable torque!\n") : printf("Failed to enable torque!\n");

    // Add parameter storage for Dynamixel present position
    dxl_addparam_result = groupSyncRead->addParam(YAW_ID);
    (dxl_addparam_result) ? printf("Succeeded to add parameter for present position!\n") : printf("Failed to add parameter for present position!\n");
    dxl_addparam_result = groupSyncRead->addParam(PITCH_ID);
    (dxl_addparam_result) ? printf("Succeeded to add parameter for present position!\n") : printf("Failed to add parameter for present position!\n");
}

int _ServoMX28::set_torque(uint8_t _id, uint8_t _flag)
{
    return packetHandler->write1ByteTxRx(portHandler, _id, ADDR_TORQUE_ENABLE, _flag, &dxl_error);
}

int _ServoMX28::disable_torque(uint8_t _id, uint8_t _flag)
{
    return packetHandler->write1ByteTxRx(portHandler, _id, ADDR_TORQUE_ENABLE, _flag, &dxl_error);
}