#include "vision/capture.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner mt(0);

    init();

    data_pub = nh.advertise<geometry_msgs::Pose>("/data", 1);
    tim = nh.createTimer(ros::Duration(0.01666666666666), cllbckTim);
    mt.spin();

    pipes.stop();
    return 0;
}

void init()
{
    // Add streams of gyro and accelerometer to configuration
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH_RGB, HEIGHT_RGB, RS2_FORMAT_RGB8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH_DEPTH, HEIGHT_DEPTH, RS2_FORMAT_Z16, 60);
    pipes.start(cfg);
}

void cllbckTim(const ros::TimerEvent &)
{
    rs2::frameset frameset = pipes.wait_for_frames();
    frameset = align_to_color.process(frameset);

    auto color = frameset.get_color_frame();
    auto depth = frameset.get_depth_frame();
    auto colorized_depth = c.colorize(depth);

    pos3D(depth);

    Mat mat_color = frame_to_mat(color);
    Mat mat_colorized_depth = frame_to_mat(colorized_depth);
    Mat mat_color_yuv;
    cvtColor(mat_color, mat_color_yuv, COLOR_BGR2YUV);

    // Get the motion frame from the frameset
    auto frame = frameset.first_or_default(RS2_STREAM_GYRO);

    // Cast the frame that arrived to motion frame
    auto motion = frame.as<rs2::motion_frame>();
    if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
    {
        double ts = motion.get_timestamp();
        rs2_vector gyro_data = motion.get_motion_data();
        process_gyro(gyro_data, ts);
    }
    if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
    {
        rs2_vector accel_data = motion.get_motion_data();
        process_accel(accel_data);
    }

    // auto motion = frame.as<rs2::pose_frame>();
    // rs2_pose theta = motion.get_pose_data();
    // printf("Theta X: %f, Theta Y: %f, Theta Z: %f\n", theta.rotation.x * RAD_TO_DEG, theta.rotation.y * RAD_TO_DEG, theta.rotation.z * RAD_TO_DEG);

    circle(mat_color, Point(240, 100), 5, Scalar(0, 255, 0), 2);
    circle(mat_colorized_depth, Point(240, 100), 5, Scalar(0, 255, 0), 2);
    imshow("YUV", mat_color_yuv);
    imshow("Color", mat_color);
    imshow("Depth", mat_colorized_depth);
    waitKey(1);
}

void pos3D(const rs2::depth_frame &frame)
{
    pixel center;
    auto pixel = get_pixel(frame);

    float vpixel[2]; // To pixel
    float vpoint[3]; // To point (in 3D)

    // Copy pixels into the arrays (to match rsutil signatures)
    vpixel[0] = static_cast<float>(pixel.first);
    vpixel[1] = static_cast<float>(pixel.second);

    // Query the frame for distance
    auto vdist = frame.get_distance(static_cast<int>(vpixel[0]), static_cast<int>(vpixel[1]));
    // printf("Vdist: %f\n", vdist);
    // Deproject from pixel to point in 3D
    rs2_intrinsics intr = frame.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data
    rs2_deproject_pixel_to_point(vpoint, &intr, vpixel, vdist);

    geometry_msgs::Pose pose;
    pose.position.x = vpoint[0];
    pose.position.y = vpoint[1];
    pose.position.z = vpoint[2];

    data_pub.publish(pose);

    printf("X: %f, Y: %f, Z: %f\n", vpoint[0], vpoint[1], vpoint[2]);
}

pixel get_pixel(const rs2::depth_frame &frame)
{
    float x = 0.55f;
    float y = 0.5f;
    int px = static_cast<int>(x * frame.get_width());
    int py = static_cast<int>(y * frame.get_height());
    return {px, py};
}

void process_gyro(rs2_vector gyro_data, double ts)
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

    data_pub.publish(angle);

    // printf("%.2f %.2f \n", gyro_angle.x * RAD_TO_DEG, gyro_data.x * RAD_TO_DEG);

    printf("Gyro Pitch: %f, Gyro Roll: %f, Gyro Yaw: %f\n", gyro_angle.x * RAD_TO_DEG, gyro_angle.z * RAD_TO_DEG, gyro_angle.y * RAD_TO_DEG);
}

void process_accel(rs2_vector accel_data)
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