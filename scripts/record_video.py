#!/usr/bin/env python3

###################################### IMPORTS #########################################

# External packages
import rospy
import rospkg
import threading
import cv2
from cv_bridge import CvBridge
import time
import subprocess
import rosservice
import numpy as np


# Services
from robot_toolkit_msgs.srv import vision_tools_srv
from perception_msgs.srv import turn_camera_srv # pylint: disable=import-error
# Messages
from robot_toolkit_msgs.msg import vision_tools_msg
from sensor_msgs.msg import Image, CompressedImage

# Import subTools
class ConsoleFormatter:
    """
    Class that represents a formatter that allows to color a text to be displayed in console.

    Attributes:
        colors (dict): Dictionary whose keys are descriptions of the value colors.
    """
    def __init__(self):
        self.colors={
        "HEADER":'\033[95m',
        "OKBLUE": '\033[94m',
        "OKGREEN": '\033[92m',
        "WARNING": '\033[93m',
        "FAIL": '\033[91m',
        "ENDC": '\033[0m'}
        
    def format(self, text, format):
        """
        Given a text and a specified format returns the text with the corresponding color for console.

        Args:
            text (str): Text to be formatted.
            format (str): Format that represents the color to be formatted.

        Raises:
            KeyError: If format is not a key in the dictionary of the attribute colors.

        Returns:
            Returns the text formatted with the color for console corresponding to the format especified.
        """
        return(self.colors[format]+text+self.colors["ENDC"])
    

class RecogUtilities:
    def __init__(self) -> None:
        # Attributes
        self.bridge = CvBridge()

        # Cameras ON/OFF
        self.front_camera_up = False
        
        # CameraImageVariables
        self.front_camera = None

        # Run in local or pepper
        self.RosOn = False
        try:
            availableServices = rosservice.get_service_list()
            self.RosOn = True
        except:
            self.RosOn = False
        
        # Run with pepper
        if self.RosOn and '/robot_toolkit/vision_tools_srv' in availableServices:
            rospy.init_node('recog_utilities')
            print(consoleFormatter.format('RUNNING RECOGNITION WITH PEPPER', 'OKGREEN'))
            
            # Service Clients
            print(consoleFormatter.format("Waiting for vision_tools service", "WARNING"))
            rospy.wait_for_service('/robot_toolkit/vision_tools_srv')
            print(consoleFormatter.format("vision_tools service connected!", "OKGREEN"))
            self.visionToolsServiceClient = rospy.ServiceProxy('/robot_toolkit/vision_tools_srv', vision_tools_srv)

            # Subscribers
            self.frontCameraCompressedSubsciber = rospy.Subscriber("/robot_toolkit_node/camera/front/image_raw/compressed", CompressedImage, self.callback_front_camera_subscriber)
        # Run with pc
        else:
            print(consoleFormatter.format('RUNNING RECOGNITION IN PC', 'OKGREEN'))
            # Init ROS
            roscore = subprocess.Popen('roscore')
            time.sleep(1)
            # Init node
            rospy.init_node('recog_utilities')
            self.frontCameraRawSubscriber = rospy.Subscriber('/camera/image_raw', Image, self.callback_local_camera_subscriber)

        # SERVICE SERVERS

        # General services
        self.turnCameraServer = rospy.Service('recog_utilities/turn_camera_srv', turn_camera_srv, self.callback_turn_camera_srv)
        print(consoleFormatter.format('turn_camera_srv on!', 'OKGREEN'))

        # Constants?
        self.FRONT_CAMERA = "front_camera"
        self.BOTTOM_CAMERA = "bottom_camera"
        self.CAMERA_DEPTH = "depth_camera"

        # PATHS
        self.PATH_RECOG_UTILITIES = rospkg.RosPack().get_path('recording_tools')
        self.PATH_DATA = self.PATH_RECOG_UTILITIES+'/resources/data/'
        self.PATH_IMAGES = self.PATH_DATA+'images/'

####################### CALLBACKS #######################
        
    # LOCAL CAMERA
    def callback_local_camera_subscriber(self, msg):
        self.front_camera = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    # PEPPER FRONT CAMERA
    def callback_front_camera_subscriber(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.front_camera = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # pylint: disable=no-member ???

####################### PUBLISHERS #######################

    # Local image publisher
    def publish_image(self):
        cap = cv2.VideoCapture(0) # pylint: disable=no-member
        pub = rospy.Publisher("camera/image_raw", Image, queue_size = 1)
        rate = rospy.Rate(10)
        bridge = CvBridge()
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                break
            msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            pub.publish(msg)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #   break
            if rospy.is_shutdown():
                cap.release()

####################### TOOLKIT CAMERA CONTROL #########################
## Service call: rosservice call /recog_utilities/turn_camera_srv "camera_name: 'front_camera'"

    def callback_turn_camera_srv(self, req):
        print(consoleFormatter.format("\nRequested turn camera service", "WARNING"))
        if self.RosOn:
            if req.camera_name in [self.FRONT_CAMERA, self.BOTTOM_CAMERA, self.CAMERA_DEPTH]:
                vision_request = vision_tools_msg()
                vision_request.camera_name = req.camera_name
                if req.command:
                    vision_request.command = req.command
                    vision_request.resolution = req.resolution #14
                    vision_request.frame_rate = req.fps
                    if req.camera_name == self.CAMERA_DEPTH:
                        vision_request.color_space = 21 #TODO averiguar bien el numero 17
                    else:
                        vision_request.color_space = 11 #TODO averiguar bien el numero 16
                else:
                    vision_request.command = "disable"

                if req.camera_name == self.BOTTOM_CAMERA:
                    self.bottom_camera_up = (req.command == "enable" or req.command == "custom")
                    print(consoleFormatter.format("The "+req.camera_name+" was "+vision_request.command, "OKBLUE"))
                elif req.camera_name == self.FRONT_CAMERA:
                    self.front_camera_up = (req.command == "enable" or req.command == "custom")
                    print(consoleFormatter.format("The "+req.camera_name+" was "+vision_request.command+"ed", "OKBLUE"))
                elif req.camera_name == self.CAMERA_DEPTH:
                    self.depth_up = (req.command == "enable" or req.command == "custom")
                    print(consoleFormatter.format("The "+req.camera_name+" was "+vision_request.command+"ed", "OKBLUE"))
                self.visionToolsServiceClient(vision_request)
                print(consoleFormatter.format('Turn camera service was executed successfully', 'OKGREEN'))
                return True
            else:
                print(consoleFormatter.format("The camera "+req.camera_name+" is not known.", "FAIL"))
                return False
        else:
            #Si no está activo en pepper:
            if req.camera_name == self.FRONT_CAMERA:
                #Prende la cámara local
                x = threading.Thread(target=self.publish_image)
                x.start()
                print(consoleFormatter.format("The "+req.camera_name+" was enabled", "OKBLUE"))
                return True
            else:
                print(consoleFormatter.format("The camera "+req.camera_name+" is not known.", "FAIL"))
                return False


if __name__ == '__main__':
    consoleFormatter=ConsoleFormatter()
    recogUtilities = RecogUtilities()
    if not recogUtilities.RosOn:
        print(consoleFormatter.format(" --- LOCAL perception_utilities node initialized --- ", "OKGREEN"))
    else:
        print(consoleFormatter.format(" --- PEPPER perception_utilities node initialized --- ", "OKGREEN"))
    rospy.spin()