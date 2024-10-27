#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <fstream>
#include <iomanip>

ros::Time last_message_time;
bool received_any_message = false;
const double TIMEOUT_SECONDS = 5.0; //timeout after 5 seconds of no messages

void writeOdometryToKittiFormat(const nav_msgs::Odometry::ConstPtr& msg, std::ofstream &outfile) {
    double px = msg->pose.pose.position.x;
    double py = msg->pose.pose.position.y;
    double pz = msg->pose.pose.position.z;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );
    tf::Matrix3x3 m(q);

    double r11 = m[0][0], r12 = m[0][1], r13 = m[0][2];
    double r21 = m[1][0], r22 = m[1][1], r23 = m[1][2];
    double r31 = m[2][0], r32 = m[2][1], r33 = m[2][2];

    //write it in a flattened KITTI format
    outfile << std::fixed << std::setprecision(6)
            << r11 << " " << r12 << " " << r13 << " " << px << " "
            << r21 << " " << r22 << " " << r23 << " " << py << " "
            << r31 << " " << r32 << " " << r33 << " " << pz << "\n";
    
    ROS_INFO("Pose written to file in KITTI format.");
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    static std::ofstream outfile("/fast_lio_ws/src/FAST_LIO/pose/output_poses.txt", std::ios::out | std::ios::app);

    if (!outfile.is_open()) {
        ROS_ERROR("Unable to open the output file");
        return;
    }

    last_message_time = ros::Time::now();
    received_any_message = true;

    writeOdometryToKittiFormat(msg, outfile);

    outfile.flush();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pose_to_kitti_node");
    ros::NodeHandle nh;

    last_message_time = ros::Time::now();

    ros::Subscriber odometry_sub = nh.subscribe("/Odometry", 1000, odometryCallback);

    ROS_INFO("pose_to_kitti_node is running and writing odometry data to KITTI format...");

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();

        if (received_any_message && (ros::Time::now() - last_message_time).toSec() > TIMEOUT_SECONDS) {
            ROS_INFO("No messages received for %.1f seconds. Shutting down...", TIMEOUT_SECONDS);
            break;
        }

        rate.sleep();
    }

    return 0;
}
