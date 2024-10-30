#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <rosbag/bag.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

rosbag::Bag bag;
bool bag_opened = false;
ros::Time last_message_time;
bool received_any_message = false;
const double TIMEOUT_SECONDS = 5.0;

//calibration transformations
tf2::Transform T_imu_to_velo;
tf2::Transform T_velo_to_cam;

void initializeImuToVeloTransform() {
    double r00 = 0.9999976, r01 = 0.0007553, r02 = -0.0020358;
    double r10 = -0.0007854, r11 = 0.9998898, r12 = -0.01482298;
    double r20 = 0.0020244, r21 = 0.01482454, r22 = 0.9998881;
    tf2::Vector3 translation(-0.8086759, 0.3195559, -0.7997231);

    tf2::Matrix3x3 rotation_matrix(r00, r01, r02, r10, r11, r12, r20, r21, r22);
    tf2::Quaternion quaternion;
    rotation_matrix.getRotation(quaternion);

    T_imu_to_velo.setOrigin(translation);
    T_imu_to_velo.setRotation(quaternion);
}

void initializeVeloToCamTransform() {
    double r00 = 0.007027555, r01 = -0.9999753, r02 = 0.00002599616;
    double r10 = -0.002254837, r11 = -0.00004184312, r12 = -0.9999975;
    double r20 = 0.9999728, r21 = 0.007027479, r22 = -0.002255075;
    tf2::Vector3 translation(-0.007137748, -0.07482656, -0.3336324);

    tf2::Matrix3x3 rotation_matrix(r00, r01, r02, r10, r11, r12, r20, r21, r22);
    tf2::Quaternion quaternion;
    rotation_matrix.getRotation(quaternion);

    T_velo_to_cam.setOrigin(translation);
    T_velo_to_cam.setRotation(quaternion);
}

// Convert IMU odometry to world frame (Camera 0)
nav_msgs::Odometry convertImuToWorldFrame(const nav_msgs::Odometry& imu_odom) {
    nav_msgs::Odometry world_odom = imu_odom;

    tf2::Transform imu_pose;
    tf2::fromMsg(imu_odom.pose.pose, imu_pose);

    tf2::Transform T_imu_to_cam = T_velo_to_cam * T_imu_to_velo;

    //for the base transformation
    tf2::Transform T_cam_to_imu = T_imu_to_cam.inverse();

    // Apply transformations: IMU -> Velodyne -> Camera 0 (World)
    tf2::Transform world_pose = T_imu_to_cam * imu_pose * T_cam_to_imu;

    geometry_msgs::Pose world_pose_msg;
    tf2::toMsg(world_pose, world_pose_msg);

    world_odom.pose.pose = world_pose_msg;
    return world_odom;
}


bool createDirectory(const std::string& path) {
    struct stat info;
    if (stat(path.c_str(), &info) != 0) {
        ROS_INFO("Directory does not exist. Creating: %s", path.c_str());
        if (mkdir(path.c_str(), 0775) == -1) {
            ROS_ERROR("Failed to create directory: %s", path.c_str());
            return false;
        }
    } else if (!(info.st_mode & S_IFDIR)) {
        ROS_ERROR("%s exists but is not a directory", path.c_str());
        return false;
    }
    return true;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!bag_opened) return;

    nav_msgs::Odometry world_odom = convertImuToWorldFrame(*msg);

    bag.write("/odometry_output", msg->header.stamp, world_odom);

    //bag.write("/odometry_output", msg->header.stamp, *msg); // for no transformation

    ROS_INFO("Converted odometry message written to bag file with timestamp: %f", msg->header.stamp.toSec());

    last_message_time = ros::Time::now();
    received_any_message = true;
}

void groundTruthCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!bag_opened) return;

    bag.write("/pose_ground_truth", msg->header.stamp, *msg);
    ROS_INFO("Ground truth pose message (PoseStamped) written to bag file with timestamp: %f", msg->header.stamp.toSec());

    last_message_time = ros::Time::now();
    received_any_message = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_to_rosbag_node");
    ros::NodeHandle nh("~");

    initializeImuToVeloTransform();
    initializeVeloToCamTransform();

    std::string bag_directory = "/fast_lio_ws/src/FAST_LIO/pose";
    if (!createDirectory(bag_directory)) {
        ROS_ERROR("Could not ensure directory exists. Exiting...");
        return 1;
    }

    std::string bag_file_path = bag_directory + "/output_poses.bag";
    try {
        bag.open(bag_file_path, rosbag::bagmode::Write);
        bag_opened = true;
        ROS_INFO("Opened bag file: %s", bag_file_path.c_str());
    } catch (const rosbag::BagException& e) {
        ROS_ERROR("Failed to open bag file: %s", e.what());
        return 1;
    }

    last_message_time = ros::Time::now();

    ros::Subscriber odometry_sub = nh.subscribe("/Odometry", 1000, odometryCallback);
    ros::Subscriber ground_truth_sub = nh.subscribe("/pose_ground_truth", 1000, groundTruthCallback);

    ROS_INFO("pose_to_rosbag_node is running and saving /Odometry and /pose_ground_truth data to bag...");

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        if (received_any_message && (ros::Time::now() - last_message_time).toSec() > TIMEOUT_SECONDS) {
            ROS_INFO("No messages received for %.1f seconds. Shutting down...", TIMEOUT_SECONDS);
            break;
        }

        rate.sleep();
    }

    if (bag_opened) {
        bag.close();
        ROS_INFO("Closed the bag file.");
    }

    return 0;
}
