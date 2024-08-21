#include "vision/capture_rs.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "capture_rs");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(0);
    image_transport::ImageTransport IT(nh);

    init();

    pub_frame_bgr = IT.advertise("/vision/realsense/frame_bgr", 1);
    pub_frame_depth = IT.advertise("/vision/realsense/frame_depth", 1);
    pub_imu_data = nh.advertise<geometry_msgs::Vector3>("/vision/realsense/imu_data", 1);

    tim_routine = nh.createTimer(ros::Duration(0.01666666666666), callbackTimer);
    spinner.spin();

    pipes.stop();
    return 0;
}

void init()
{
    // Add streams of gyro and accelerometer to configuration
    // cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH_RGB, HEIGHT_RGB, RS2_FORMAT_RGB8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH_DEPTH, HEIGHT_DEPTH, RS2_FORMAT_Z16, 60);
    pipes.start(cfg);
}

void callbackTimer(const ros::TimerEvent &)
{
    rs2::frameset frameset = pipes.wait_for_frames();
    frameset = align_to_color.process(frameset);

    // Get the color and depth frames from the frameset
    auto color = frameset.get_color_frame();
    auto depth = frameset.get_depth_frame();
    auto colorized_depth = c.colorize(depth);

    // Convert the frames to OpenCV Mat
    Mat mat_color = frameToMat(color);
    Mat mat_colorized_depth = frameToMat(colorized_depth);

    // Get the motion frame from the frameset
    auto frame = frameset.first_or_default(RS2_STREAM_GYRO);

    // Cast the frame that arrived to motion frame
    auto motion = frame.as<rs2::motion_frame>();
    if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
    {
        double ts = motion.get_timestamp();
        rs2_vector gyro_data = motion.get_motion_data();
        processGyro(gyro_data, ts);
    }

    if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
    {
        rs2_vector accel_data = motion.get_motion_data();
        processAccel(accel_data);
    }

    // Publish the frames
    sensor_msgs::ImagePtr msg_color = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_color).toImageMsg();
    sensor_msgs::ImagePtr msg_colorized_depth = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_colorized_depth).toImageMsg();
    pub_frame_bgr.publish(msg_color);
    pub_frame_depth.publish(msg_colorized_depth);
}

void processGyro(rs2_vector gyro_data, double ts)
{
    if (firstGyro) // On the first iteration, use only data from accelerometer to set the camera's initial position
    {
        firstGyro = false;
        last_ts_gyro = ts;
        return;
    }
    // Holds the change in angle, as calculated from gyro
    static float3 gyro_angle;

    // Compute the difference between arrival times of previous and current gyro frames
    double dt_gyro = (ts - last_ts_gyro) / 1000.0;
    last_ts_gyro = ts;

    // Change in angle equals gyro measures * time passed since last measurement
    gyro_angle.x += gyro_data.x * static_cast<float>(dt_gyro); // Pitch
    gyro_angle.y += gyro_data.y * static_cast<float>(dt_gyro); // Yaw
    gyro_angle.z += gyro_data.z * static_cast<float>(dt_gyro); // Roll

    static tf2::Quaternion quat;
    quat.setRPY(gyro_angle.z, gyro_angle.x, gyro_angle.y);

    static geometry_msgs::Pose angle;
    angle.orientation.x = quat.x();
    angle.orientation.y = quat.y();
    angle.orientation.z = quat.z();
    angle.orientation.w = quat.w();

    pub_imu_data.publish(angle);

    printf("Gyro Pitch: %f, Gyro Roll: %f, Gyro Yaw: %f\n", gyro_angle.x * RAD_TO_DEG, gyro_angle.z * RAD_TO_DEG, gyro_angle.y * RAD_TO_DEG);
}

void processAccel(rs2_vector accel_data)
{
    // theta is the angle of camera rotation in x, y and z components
    float3 theta;
    std::mutex theta_mtx;
    /* alpha indicates the part that gyro and accelerometer take in computation of theta; higher alpha gives more weight to gyro, but too high
    values cause drift; lower alpha gives more weight to accelerometer, which is more sensitive to disturbances */
    float alpha = 0.98f;
    bool firstGyro = true;
    bool firstAccel = true;
    // Keeps the arrival time of previous gyro frame
    double last_ts_gyro = 0;
    // Holds the angle as calculated from accelerometer data
    float3 accel_angle;
    // printf("Accel Pitch: %f, Accel Roll: %f\n", accel_data.x * RAD_TO_DEG, accel_data.z * RAD_TO_DEG);

    // Calculate rotation angle from accelerometer data
    accel_angle.z = atan2(accel_data.y, accel_data.z);
    accel_angle.x = atan2(accel_data.x, sqrt(accel_data.y * accel_data.y + accel_data.z * accel_data.z));

    // If it is the first iteration, set initial pose of camera according to accelerometer data (note the different handling for Y axis)
    std::lock_guard<std::mutex> lock(theta_mtx);
    if (firstAccel)
    {
        firstAccel = false;
        theta = accel_angle;
        // Since we can't infer the angle around Y axis using accelerometer data, we'll use PI as a convetion for the initial pose
        theta.y = PI_FL;
    }
    else
    {
        /*
        Apply Complementary Filter:
            - high-pass filter = theta * alpha:  allows short-duration signals to pass through while filtering out signals
              that are steady over time, is used to cancel out drift.
            - low-pass filter = accel * (1- alpha): lets through long term changes, filtering out short term fluctuations
        */
        theta.x = theta.x * alpha + accel_angle.x * (1 - alpha);
        theta.z = theta.z * alpha + accel_angle.z * (1 - alpha);
    }
    printf("Accel Pitch: %f, Accel Roll: %f\n", accel_angle.x * RAD_TO_DEG, accel_angle.z * RAD_TO_DEG);
}

Mat frameToMat(const rs2::frame &f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r_rgb = Mat(Size(w, h), CV_8UC3, (void *)f.get_data(), Mat::AUTO_STEP);
        Mat r_bgr;
        cvtColor(r_rgb, r_bgr, COLOR_RGB2BGR);
        return r_bgr;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void *)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void *)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void *)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

Mat depthFrameToMeters(const rs2::depth_frame &f)
{
    cv::Mat dm = frameToMat(f);
    dm.convertTo(dm, CV_64F);
    dm = dm * f.get_units();
    return dm;
}