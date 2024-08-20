#include "hardware/servo_mx28.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "servo_mx28");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(0);

    //--Initialize Servo
    _ServoMX28::getInstance().servo_mx28_init();

    //--Subscriber
    rpy_sub = nh.subscribe("/master/servo_goal_pose", 1, &_ServoMX28::rpy_callback);

    //--Publisher
    rpy_pub = nh.advertise<auv_msgs::RPY>("/hardware/servo_current_pose", 1);

    //--Timer
    servo_routine = nh.createTimer(ros::Duration(0.01), &_ServoMX28::servo_routine_callback, &_ServoMX28::getInstance());
    spinner.spin();
    return 0;
}

void _ServoMX28::rpy_callback(const auv_msgs::RPY::ConstPtr& msg)
{
    yaw_goal_position = msg->yaw * DEG2POSITION;
    pitch_goal_position = msg->pitch * DEG2POSITION;
}

void _ServoMX28::servo_routine_callback(const ros::TimerEvent& event)
{
    param_yaw_goal_position[0] = DXL_LOBYTE(yaw_goal_position);
    param_yaw_goal_position[1] = DXL_HIBYTE(yaw_goal_position);
    param_pitch_goal_position[0] = DXL_LOBYTE(pitch_goal_position);
    param_pitch_goal_position[1] = DXL_HIBYTE(pitch_goal_position);

    // Add Dynamixel goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite->addParam(ID_YAW, param_yaw_goal_position);
    if (dxl_addparam_result != true) {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", YAW_ID);
        return 0;
    }

    dxl_addparam_result = groupSyncWrite->addParam(ID_PITCH, param_pitch_goal_position);
    if (dxl_addparam_result != true) {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", PITCH_ID);
        return 0;
    }

    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite->txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }

    // Clear syncwrite parameter storage
    groupSyncWrite->clearParam();

    // Syncread present position
    dxl_comm_result = groupSyncRead->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }

    // Check if groupSyncRead data of Dynamixel present position is available
    dxl_getdata_result = groupSyncRead->isAvailable(ID_YAW, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    if (dxl_getdata_result != true) {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", YAW_ID);
        return 0;
    }

    dxl_getdata_result = groupSyncRead->isAvailable(ID_PITCH, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    if (dxl_getdata_result != true) {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", PITCH_ID);
        return 0;
    }

    // Get present position value
    yaw_present_position = groupSyncRead->getData(ID_YAW, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    pitch_present_position = groupSyncRead->getData(ID_PITCH, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

    auv_msgs::RPY rpy_msg;
    rpy_msg.yaw = yaw_present_position * POSITION2DEG;
    rpy_msg.pitch = pitch_present_position * POSITION2DEG;

    if (rpy_msg.yaw > 180)
        rpy_msg.yaw -= 360;
    else if (rpy_msg.yaw < -180)
        rpy_msg.yaw += 360;

    if (rpy_msg.pitch > 180)
        rpy_msg.pitch -= 360;
    else if (rpy_msg.pitch < -180)
        rpy_msg.pitch += 360;

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
    dxl_comm_result[0] = set_torque(ID_YAW, ENABLE);
    dxl_comm_result[1] = set_torque(ID_PITCH, ENABLE);

    // Add parameter storage for Dynamixel present position
    dxl_addparam_result = groupSyncRead->addParam(ID_YAW);
    (dxl_addparam_result) ? printf("Succeeded to add parameter for present position!\n") : printf("Failed to add parameter for present position!\n");
    dxl_addparam_result = groupSyncRead->addParam(ID_PITCH);
    (dxl_addparam_result) ? printf("Succeeded to add parameter for present position!\n") : printf("Failed to add parameter for present position!\n");
}

int _ServoMX28::set_torque(uint8_t _id, uint8_t _flag)
{
    return packetHandler->write1ByteTxRx(portHandler, _id, ADDR_TORQUE_ENABLE, _flag, &dxl_error[_id - 1]);
}

int _ServoMX28::disable_torque(uint8_t _id, uint8_t _flag)
{
    return packetHandler->write1ByteTxRx(portHandler, _id, ADDR_TORQUE_ENABLE, _flag, &dxl_error[_id - 1]);
}