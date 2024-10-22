#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <iomanip>

// Global variable to track the last message time
ros::Time last_message_time;
bool received_any_message = false;
const double TIMEOUT_SECONDS = 5.0; // Timeout after 5 seconds of no messages

void writeOdometryToKittiFormat(const nav_msgs::Odometry::ConstPtr& msg, std::ofstream &outfile) {
    // Extract the position
    double px = msg->pose.pose.position.x;
    double py = msg->pose.pose.position.y;
    double pz = msg->pose.pose.position.z;

    // Extract the orientation (quaternion) and convert to a 3x3 rotation matrix
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);

    // Rotation matrix (3x3)
    double r11 = m[0][0], r12 = m[0][1], r13 = m[0][2];
    double r21 = m[1][0], r22 = m[1][1], r23 = m[1][2];
    double r31 = m[2][0], r32 = m[2][1], r33 = m[2][2];

    // Write the 3x4 matrix in KITTI format (flattened)
    outfile << std::fixed << std::setprecision(6)
            << r11 << " " << r12 << " " << r13 << " " << px << " "
            << r21 << " " << r22 << " " << r23 << " " << py << " "
            << r31 << " " << r32 << " " << r33 << " " << pz << "\n";
    
    // Log a message
    ROS_INFO("Pose written to file in KITTI format.");
}

// Callback function to handle Odometry messages
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    static std::ofstream outfile("/fast_lio_ws/src/FAST_LIO/pose/output_poses.txt", std::ios::out | std::ios::app);

    if (!outfile.is_open()) {
        ROS_ERROR("Unable to open the output file");
        return;
    }

    // Update the last message time
    last_message_time = ros::Time::now();
    received_any_message = true;

    // Write the odometry data to the file in KITTI format
    writeOdometryToKittiFormat(msg, outfile);

    // Flush the file to ensure the data is saved immediately
    outfile.flush();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_to_kitti_node");
    ros::NodeHandle nh;

    // Initialize the last message time to the current time
    last_message_time = ros::Time::now();

    // Subscribe to the /Odometry topic
    ros::Subscriber odometry_sub = nh.subscribe("/Odometry", 1000, odometryCallback);

    // Log that the node is running
    ROS_INFO("pose_to_kitti_node is running and writing odometry data to KITTI format...");

    // Main loop with timeout check
    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        ros::spinOnce();

        // Check for timeout if we've already received at least one message
        if (received_any_message && (ros::Time::now() - last_message_time).toSec() > TIMEOUT_SECONDS) {
            ROS_INFO("No messages received for %.1f seconds. Shutting down...", TIMEOUT_SECONDS);
            break; // Exit the loop to shut down
        }

        rate.sleep();
    }

    return 0;
}
