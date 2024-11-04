#!/usr/bin/env python3

import os
import sys

import cv2
import rosbag
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage


def mp4_to_bag(
    mp4_file_path,
    frame_rate_of_video=30,
    hz_of_rosbag=10,
    start_time=0,
    duration=0,
    topic_name="/camera/image_raw/compressed",
):
    # Print the parameters
    print()
    print("Converting video to rosbag file with the following parameters:")
    print(f"\tmp4_file_path: {mp4_file_path}")
    print(f"\tframe_rate_of_video: {frame_rate_of_video}")
    print(f"\thz_of_rosbag: {hz_of_rosbag}")
    print(f"\tstart_time: {start_time}")
    if duration > 0:
        print(f"\tduration: {duration}")
    print(f"\ttopic_name: {topic_name}")
    print()

    # Open the video file
    cap = cv2.VideoCapture(mp4_file_path)
    if not cap.isOpened():
        print(f"Error: Could not open video file {mp4_file}")
        return

    # Initialize ROS node
    rospy.init_node("mp4_to_bag", anonymous=True)

    # Create a bag file
    bag_file_path = mp4_file_path.replace(".mp4", ".bag")
    bag = rosbag.Bag(bag_file_path, "w")

    # check topic name
    if "compressed" not in topic_name:
        if not topic_name.endswith("/"):
            topic_name += "/"
        topic_name += "compressed"

    # Initialize CvBridge
    bridge = CvBridge()

    # Write the frames to the bag file
    frame_id = 0
    time_stamp_ros = start_time_ros = rospy.Time.now()
    ratio = frame_rate_of_video // hz_of_rosbag
    print("Start writing frames to bag file ", end="", flush=True)
    while (
        cap.isOpened()
        and not rospy.is_shutdown()
        or (
            duration > 0
            and (time_stamp_ros - start_time_ros).to_sec()
            > (start_time + duration)
        )
    ):
        ret, frame = cap.read()
        if not ret:
            break

        if (frame_id % ratio) == 0 and (
            (time_stamp_ros - start_time_ros).to_sec() > start_time
        ):
            # Convert the frame to a ROS Image message
            img_msg = bridge.cv2_to_compressed_imgmsg(frame, dst_format="jpg")
            img_msg.header.stamp = time_stamp_ros
            img_msg.header.frame_id = str(frame_id // ratio)

            # Write the Image message to the bag file
            bag.write(topic_name, img_msg, img_msg.header.stamp)
            print(".", end="", flush=True)

        frame_id += 1
        time_stamp_ros += rospy.Duration(1 / frame_rate_of_video)

    print("Done\n")

    # Release the video capture and close the bag file
    cap.release()
    bag.close()
    print(f"Succesfully wrote {frame_id // ratio} frames to {bag_file_path}")


if __name__ == "__main__":
    args = sys.argv
    if len(args) < 4:
        print(
            "usage: rosrun video_to_rosbag video_to_rosbag.py <mp4_file_path> <frame_rate_of_video> <hz_of_rosbag> [start_time] [duration] [topic_name]"
        )
        sys.exit(1)

    mp4_file_path = args[1]
    frame_rate_of_video = int(args[2])
    hz_of_rosbag = int(args[3])
    start_time = float(args[4]) if len(args) > 4 else 0
    duration = float(args[5]) if len(args) > 5 else 0
    topic_name = args[6] if len(args) > 6 else "/camera/image_raw/compressed"

    if not os.path.isfile(mp4_file_path):
        print(f"Error: File {mp4_file_path} does not exist")
        sys.exit(1)

    mp4_to_bag(
        mp4_file_path=mp4_file_path,
        frame_rate_of_video=frame_rate_of_video,
        hz_of_rosbag=hz_of_rosbag,
        start_time=start_time,
        duration=duration,
        topic_name=topic_name,
    )
