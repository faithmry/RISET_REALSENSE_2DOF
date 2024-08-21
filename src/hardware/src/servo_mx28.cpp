#include "hardware/servo_mx28.hpp"

Servo_MX28* servo = Servo_MX28::getInstance();

int main(int argc, char** argv)
{
    ros::init(argc, argv, "servo_mx28");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(0);

    //--Initialize
    servo->init(&nh);

    spinner.spin();
    return 0;
}

void Servo_MX28::init(ros::NodeHandle* nh)
{
    initMX28();

    //--ROS Publisher
    pub_servo2pc = nh->advertise<geometry_msgs::Vector3>("/hardware/servo_current_pose", 1);

    //--ROS Subscriber
    sub_pc2servo = nh->subscribe("/master/servo_goal_pose", 1, &Servo_MX28::callbackSubscribeRPY, this);

    //--ROS Timer
    tim_routine = nh->createTimer(ros::Duration(0.01), &Servo_MX28::callbackRoutine, this);
}

void Servo_MX28::initMX28()
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
    dxl_comm_result = setTorque(YAW_ID, ENABLE);
    (dxl_comm_result == COMM_SUCCESS) ? printf("Succeeded to enable torque!\n") : printf("Failed to enable torque!\n");
    dxl_comm_result = setTorque(PITCH_ID, ENABLE);
    (dxl_comm_result == COMM_SUCCESS) ? printf("Succeeded to enable torque!\n") : printf("Failed to enable torque!\n");

    // Add parameter storage for Dynamixel present position
    dxl_addparam_result = groupSyncRead->addParam(YAW_ID);
    (dxl_addparam_result) ? printf("Succeeded to add parameter for present position!\n") : printf("Failed to add parameter for present position!\n");
    dxl_addparam_result = groupSyncRead->addParam(PITCH_ID);
    (dxl_addparam_result) ? printf("Succeeded to add parameter for present position!\n") : printf("Failed to add parameter for present position!\n");
}

void Servo_MX28::callbackSubscribeRPY(const geometry_msgs::Vector3::ConstPtr& msg)
{
    goal_position.clear();
    goal_position = jointConvertToPosition({ msg->z, msg->y });
}

void Servo_MX28::callbackRoutine(const ros::TimerEvent& event)
{
    writeGoalPosition();
    readPresentPosition();

    std::vector<double> temp_rpy;
    temp_rpy = jointConvertToDegree(present_position);

    geometry_msgs::Vector3 rpy_msg;
    rpy_msg.z = temp_rpy.at(0);
    rpy_msg.y = temp_rpy.at(1);

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

void Servo_MX28::writeGoalPosition()
{
    memcpy(param_yaw_goal_position, &goal_position.at(0), sizeof(goal_position.at(0)));
    memcpy(param_pitch_goal_position, &goal_position.at(1), sizeof(goal_position.at(1)));
    dxl_addparam_result = groupSyncWrite->addParam(YAW_ID, param_yaw_goal_position);
    dxl_addparam_result = groupSyncWrite->addParam(PITCH_ID, param_pitch_goal_position);
    dxl_comm_result = groupSyncWrite->txPacket();
    groupSyncWrite->clearParam();
}

void Servo_MX28::readPresentPosition()
{
    dxl_comm_result = groupSyncRead->txRxPacket();
    present_position.at(0) = groupSyncRead->getData(YAW_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    present_position.at(1) = groupSyncRead->getData(PITCH_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
}

int8_t Servo_MX28::setTorque(uint8_t _id, uint8_t _flag)
{
    return packetHandler->write1ByteTxRx(portHandler, _id, ADDR_TORQUE_ENABLE, _flag, &dxl_error);
}

std::vector<uint16_t> Servo_MX28::jointConvertToPosition(std::vector<double> joint_angle)
{
    if (joint_angle.size() != 2) {
        printf("Invalid joint angle size\n");
        return std::vector<uint16_t>();
    }

    return { YAW_OFFSET_POSITION + joint_angle[0] DEG2POSITION, PITCH_OFFSET_POSITION + joint_angle[1] DEG2POSITION };
}

std::vector<double> Servo_MX28::jointConvertToDegree(std::vector<uint16_t> joint_position)
{
    if (joint_position.size() != 2) {
        printf("Invalid joint position size\n");
        return std::vector<double>();
    }

    return { (joint_position[0] - YAW_OFFSET_POSITION) POSITION2DEG, (joint_position[1] - PITCH_OFFSET_POSITION) POSITION2DEG };
}
