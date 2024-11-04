# video_to_rosbag

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Convert an MP4 video file to a ROS bag file

## Install and Build
```bash
# clone repository
cd /path/to/your/catkin_ws/src
git clone https://github.com/ToshikiNakamura0412/video_to_rosbag.git

# build
cd /path/to/your/catkin_ws
rosdep install -riy --from-paths src --rosdistro noetic # Install dependencies
catkin build video_to_rosbag
```

## Usage
```bash
roscore
rosrun video_to_rosbag video_to_rosbag.py <mp4_file_path> <frame_rate_of_video> <hz_of_rosbag> [start_time] [duration] [topic_name]
```

## Example
```bash
> rosrun video_to_rosbag video_to_rosbag.py ~/bagfiles/sample.mp4 30 10

Converting video to rosbag file with the following parameters:
        mp4_file_path: /home/user/bagfiles/sample.mp4
        frame_rate_of_video: 30
        hz_of_rosbag: 10
        start_time: 0
        topic_name: /camera/image_raw/compressed

Start writing frames to bag file .......................................................................................................................................................................................................................................................................................................................................................................................................................Done

Succesfully wrote 408 frames to /home/user/bagfiles/sample.bag
```
