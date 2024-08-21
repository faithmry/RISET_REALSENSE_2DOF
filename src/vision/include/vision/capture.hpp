#include "vision/cv-helpers.hpp"
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "bits/stdc++.h"
#include "geometry_msgs/Pose.h"
#include <tf2/LinearMath/Quaternion.h>

using namespace cv;
using namespace rs2;
using namespace std;
using pixel = pair<int, int>;

#define WIDTH_DEPTH 480
#define HEIGHT_DEPTH 270
#define WIDTH_RGB 424
#define HEIGHT_RGB 240
#define RAD_TO_DEG 57.2957795f
#define PI_FL 3.141592f

ros::Publisher data_pub;
ros::Timer tim;
rs2::config cfg;
rs2::pipeline pipes;
rs2::colorizer c;
rs2::align align_to_color(RS2_STREAM_COLOR);

bool firstGyro = true;
// Keeps the arrival time of previous gyro frame
double last_ts_gyro = 0;

struct float3
{
    float x, y, z;
};

void init();
void cllbckTim(const ros::TimerEvent &);
void pos3D(const rs2::depth_frame &frame);
pixel get_pixel(const rs2::depth_frame &frame);
void process_gyro(rs2_vector gyro_data, double ts);
void process_accel(rs2_vector accel_data);