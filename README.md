# recording_tools
* A simple package for recording audio and video from local pc or the robot pepper.
* ROS Packages needes: **perception_msgs**, **robot_toolkit_msgs**
If running from local:
* rosrun recording_tools record_video.py
* rosservice call /recog_utilities/turn_camera_srv "camera_name: 'front_camera'"
* rosrun recording_tools img_subs.py <seconds/ornot> 
