#!/usr/bin/env python

import rosbag
import sensor_msgs.point_cloud2 as pc2
from collections import defaultdict
from tqdm import tqdm
import argparse

def label_counter(rosbag_path, topic_name):
    """
    Counts the labels in the point cloud data from a ROS bag file.

    Args:
        rosbag_path (str): Path to the ROS bag file.
        topic_name (str): Topic name containing the point cloud data.

    Returns:
        dict: A dictionary with labels as keys and their counts as values.
    """
    # Dictionary to hold the label counts
    label_counts = defaultdict(int)
    
    print(f"Opening ROS bag: {rosbag_path}")
    with rosbag.Bag(rosbag_path, 'r') as bag:
        for topic, msg, _ in tqdm(bag.read_messages(topics=[topic_name]), desc="Reading messages"):
            if topic == topic_name:
                # Parse the point cloud data
                for point in pc2.read_points(msg, field_names=["label"], skip_nans=True):
                    label = int(point[0]) & 0xFFFF  # Ensure the label is interpreted as uint16
                    label_counts[label] += 1

    print("Label counting complete.")
    return label_counts

if __name__ == "__main__":
    # Argument parser
    parser = argparse.ArgumentParser(description="Count labels in a specified ROS bag file.")
    parser.add_argument("filename", type=str, help="Name of the ROS bag file to process.")
    args = parser.parse_args()

    # Construct the path using the filename
    rosbag_path = f"/root/semantickitti2bag2/output/{args.filename}"
    topic_name = "/points_raw"

    # Run the label counting function
    label_counts = label_counter(rosbag_path, topic_name)
    
    # Print the results
    print("Label counts:")
    for label, count in sorted(label_counts.items()):
        print(f"Label {label}: {count}")
