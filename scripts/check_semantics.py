#!/usr/bin/env python

import rosbag
import sensor_msgs.point_cloud2 as pc2
from collections import defaultdict
from tqdm import tqdm
import argparse

def count_labels_in_bag(rosbag_path, topic_name):
    """Count the occurrences of each label in the specified topic of the ROS bag."""
    label_counts = defaultdict(int)  # To store the count of each label

    print(f"Opening ROS bag: {rosbag_path}")
    with rosbag.Bag(rosbag_path, 'r') as bag:
        # Verify the topic exists and is of the correct type
        topics_info = bag.get_type_and_topic_info().topics
        if topic_name not in topics_info or topics_info[topic_name].msg_type != 'sensor_msgs/PointCloud2':
            print(f"The topic '{topic_name}' either does not exist or is not of type 'sensor_msgs/PointCloud2'.")
            return

        print(f"Processing topic: {topic_name}")
        # Read and process each message in the topic
        for _, msg, _ in tqdm(bag.read_messages(topics=[topic_name]), desc="Processing messages"):
            # Extract points, including the 'label' field
            points = pc2.read_points(msg, field_names=['label'], skip_nans=True)
            for point in points:
                label = int(point[0])  # Extract the 'label' value from the tuple
                label_counts[label] += 1

    print("\nLabel counts in the dataset:")
    for label, count in sorted(label_counts.items()):
        print(f"Label {label}: {count} occurrences")

if __name__ == "__main__":
    # Argument parser
    parser = argparse.ArgumentParser(description="Count labels in a specified ROS bag file.")
    parser.add_argument("filename", type=str, help="Name of the ROS bag file to process.")
    args = parser.parse_args()

    # Construct the path using the filename
    rosbag_path = f"/root/semantickitti2bag2/output/{args.filename}"
    topic_name = "/points_raw"

    # Run the label counting function
    count_labels_in_bag(rosbag_path, topic_name)
