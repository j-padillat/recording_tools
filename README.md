# recording_tools
* A simple package for recording audio and video from local pc or the robot pepper.
* ROS Packages needes: **perception_msgs**, **robot_toolkit_msgs**
If running from local:
* roslaunch recording_tools recording_tools.launch
* rosservice call /recog_utilities/turn_camera_srv "camera_name: 'front_camera'"
* rosrun recording_tools multimedia_recorder.py <seconds/or_not> 

If running with pepper:
* roslaunch recording_tools recording_tools.launch
* rosservice call /recog_utilities/turn_camera_srv 'front_camera' 'enable' 14 10
* rosrun recording_tools multimedia_recorder.py <seconds/or_not> 