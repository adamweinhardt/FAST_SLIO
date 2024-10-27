#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <rosbag/bag.h>
#include <sys/stat.h>
#include <sys/types.h>

rosbag::Bag bag;
bool bag_opened = false;
ros::Time last_message_time;
bool received_any_message = false;
const double TIMEOUT_SECONDS = 5.0;

std::string output_format; //Can be "PoseStamped" or "Odometry"

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

    if (output_format == "PoseStamped") {
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.pose = msg->pose.pose;

        bag.write("/pose_data", ros::Time::now(), pose_msg);
        ROS_INFO("PoseStamped message written to bag file.");
    } 
    else if (output_format == "Odometry") {
        bag.write("/odometry_data", ros::Time::now(), *msg);
        ROS_INFO("Odometry message written to bag file.");
    } 
    else {
        ROS_ERROR("Invalid output format specified. Use either 'PoseStamped' or 'Odometry'.");
    }

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

    nh.param("output_format", output_format, std::string("Odometry"));  // Default to "Odometry"
    ROS_INFO("Output format set to: %s", output_format.c_str());

    last_message_time = ros::Time::now();

    ros::Subscriber odometry_sub = nh.subscribe("/Odometry", 1000, odometryCallback);

    ROS_INFO("pose_to_rosbag_node is running and saving odometry data to bag in %s format...", output_format.c_str());

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
