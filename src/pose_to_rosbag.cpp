#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <sys/stat.h>
#include <sys/types.h>

rosbag::Bag bag;
bool bag_opened = false;
ros::Time last_message_time;
bool received_any_message = false;
const double TIMEOUT_SECONDS = 5.0;

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

    bag.write("/odometry_output", ros::Time::now(), *msg);
    ROS_INFO("Odometry message (pipeline output) written to bag file.");

    last_message_time = ros::Time::now();
    received_any_message = true;
}

void groundTruthCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    if (!bag_opened) return;

    bag.write("/ground_truth_pose", ros::Time::now(), *msg);
    ROS_INFO("Ground truth pose message written to bag file.");

    last_message_time = ros::Time::now();
    received_any_message = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_to_rosbag_node");
    ros::NodeHandle nh("~");

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

    //subscribe to odometry output and ground truth pose topics
    ros::Subscriber odometry_sub = nh.subscribe("/Odometry", 1000, odometryCallback);
    ros::Subscriber ground_truth_sub = nh.subscribe("/ground_truth", 1000, groundTruthCallback);

    ROS_INFO("pose_to_rosbag_node is running and saving /Odometry and /ground_truth data to bag...");

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
