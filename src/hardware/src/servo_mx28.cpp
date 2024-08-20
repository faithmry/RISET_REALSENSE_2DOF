#include "hardware/servo_mx28.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_mx28");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(0);

    //--Initialize Servo
    _ServoMX28::getInstance().servo_mx28_init();

    //--Timer
    servo_routine = nh.createTimer(ros::Duration(1.1), &_ServoMX28::servo_routine_callback, &_ServoMX28::getInstance());
    spinner.spin();
    return 0;
}

void _ServoMX28::servo_routine_callback(const ros::TimerEvent &event)
{
    // Write Goal Position
    packetHandler->write4ByteTxRx(portHandler, ID_YAW, ADDR_GOAL_POSITION, dxl_goal_position[0][index], &dxl_error[0]);
    packetHandler->write4ByteTxRx(portHandler, ID_PITCH, ADDR_GOAL_POSITION, dxl_goal_position[1][index], &dxl_error[1]);

    // Read Present Position
    // do
    // {
    dxl_comm_result[0] = packetHandler->read4ByteTxRx(portHandler, ID_YAW, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position[0], &dxl_error[0]);
    printf("[ID:%03d] Goal Position:%03d  Present Position:%03d\n", ID_YAW, dxl_goal_position[0][index], dxl_present_position[0]);
    // } while ((abs(dxl_goal_position[0][index] - dxl_present_position[0]) > MOVING_STATUS_THRESHOLD));

    // do
    // {
    dxl_comm_result[1] = packetHandler->read4ByteTxRx(portHandler, ID_PITCH, ADDR_PRESENT_POSITION, (uint32_t *)&dxl_present_position[1], &dxl_error[1]);
    printf("[ID:%03d] Goal Position:%03d  Present Position:%03d\n", ID_PITCH, dxl_goal_position[1][index], dxl_present_position[1]);
    // } while ((abs(dxl_goal_position[1][index] - dxl_present_position[1]) > MOVING_STATUS_THRESHOLD));

    // Switch the Goal Position
    index = (index == 0) ? 1 : 0;
}

void _ServoMX28::servo_mx28_init()
{
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    index = 0;
    dxl_comm_result[0] = COMM_TX_FAIL;       // Communication result
    dxl_comm_result[1] = COMM_TX_FAIL;       // Communication result
    dxl_error[0] = 0;                        // DYNAMIXEL error
    dxl_error[1] = 0;                        // DYNAMIXEL error
    dxl_goal_position[0][0] = YAW_MINIMUM;   // Goal position
    dxl_goal_position[0][1] = YAW_MAXIMUM;   // Goal position
    dxl_goal_position[1][0] = PITCH_MINIMUM; // Goal position
    dxl_goal_position[1][1] = PITCH_MAXIMUM; // Goal position
    dxl_present_position[0] = 0;             // Read 4 byte Position data
    dxl_present_position[1] = 0;             // Read 4 byte Position data

    // Open port
    (portHandler->openPort()) ? printf("Succeeded to open the port!\n") : printf("Failed to open the port!\n");

    // Set port baudrate
    (portHandler->setBaudRate(BAUDRATE)) ? printf("Succeeded to change the baudrate!\n") : printf("Failed to change the baudrate!\n");

    // Enable DYNAMIXEL Torque
    dxl_comm_result[0] = set_torque(ID_YAW, ENABLE);
    dxl_comm_result[1] = set_torque(ID_PITCH, ENABLE);
}

int _ServoMX28::set_torque(uint8_t _id, uint8_t _flag)
{
    return packetHandler->write1ByteTxRx(portHandler, _id, ADDR_TORQUE_ENABLE, _flag, &dxl_error[_id - 1]);
}

int _ServoMX28::disable_torque(uint8_t _id, uint8_t _flag)
{
    return packetHandler->write1ByteTxRx(portHandler, _id, ADDR_TORQUE_ENABLE, _flag, &dxl_error[_id - 1]);
}