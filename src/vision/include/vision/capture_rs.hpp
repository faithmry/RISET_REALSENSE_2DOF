#ifndef CAPTURE_RS_HPP
#define CAPTURE_RS_HPP

//--ROS Headers
#include <ros/package.h>
#include <ros/ros.h>

//--ROS Messages
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>
#include <tf2/LinearMath/Quaternion.h>

//--OpenCV Headers
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

//--C++ Headers
#include <exception>
#include <yaml-cpp/yaml.h>

//--RealSense Headers
#include <librealsense2/rs.hpp>

#define WIDTH_DEPTH 480
#define HEIGHT_DEPTH 270
#define WIDTH_RGB 424
#define HEIGHT_RGB 240
#define RAD_TO_DEG 57.2957795f
#define PI_FL 3.141592f

using namespace cv;
using namespace rs2;
using namespace std;

//--Publisher
image_transport::Publisher pub_frame_bgr;
image_transport::Publisher pub_frame_depth;
ros::Publisher pub_imu_data;

//--Services

//--Subscriber

//--Timer
ros::Timer tim_routine;

rs2::config cfg;
rs2::pipeline pipes;
rs2::colorizer c;
rs2::align align_to_color(RS2_STREAM_COLOR);

bool firstGyro = true;
double last_ts_gyro = 0;

struct float3 {
    float x, y, z;
};

//--Prototypes
void callbackTimer(const ros::TimerEvent&);

void init();
void processGyro(rs2_vector gyro_data, double ts);
void processAccel(rs2_vector accel_data);

Mat frameToMat(const rs2::frame& f);
Mat depthFrameToMeters(const rs2::depth_frame& f);

#endif