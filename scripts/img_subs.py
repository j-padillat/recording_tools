#!/usr/bin/env python3
from __future__ import print_function

import rospkg
import sys
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import time
import rostopic
import numpy as np

class image_converter:

  def __init__(self, segundos=8):

    self.RosOn = False
    try:
        availableTopics = rostopic.get_topic_list()
        self.RosOn = True
    except:
        self.RosOn = False

    self.bridge = CvBridge()
    if self.RosOn and '/robot_toolkit_node/camera/front/image_raw/compressed' in availableTopics:
        self.image_sub = rospy.Subscriber("/robot_toolkit_node/camera/front/image_raw/compressed", CompressedImage, self.callback_pepper)
    else:
        self.image_sub = rospy.Subscriber("camera/image_raw", Image, self.callback_local)
    self.tiempo_inicial = time.time()
    self.img_array = []
    self.counter = 0
    self.seconds = segundos
    self.PATH_RECOG_UTILITIES = rospkg.RosPack().get_path('recording_tools')
    self.PATH_VIDEOS = self.PATH_RECOG_UTILITIES+'/resources/videos/'

  def callback_local(self,data):
    self.actual_time = time.time()
    if (self.actual_time - self.tiempo_inicial <= self.seconds):
        self.counter +=1
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        self.img_array.append(cv_image)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
    else:
        self.height, self.width, self.layers = self.img_array[-1].shape
        self.size = (self.width, self.height)

        self.hora = time.localtime()
        self.hora_string = time.strftime("%H:%M:%S", self.hora)
        self.hora_string = self.hora_string.replace(':','_')
    
        out = cv2.VideoWriter(self.PATH_VIDEOS+'video_{}.avi'.format(self.hora_string),cv2.VideoWriter_fourcc(*'MJPG'), 10, self.size)
 
        for i in range(len(self.img_array)):
            out.write(self.img_array[i])
        out.release()
        reason = "Termino la grabacion, numero de frames: {}".format(self.counter)
        print(reason)
        rospy.signal_shutdown(reason)

def callback_pepper(self,msg):
    self.actual_time = time.time()
    if (self.actual_time - self.tiempo_inicial <= self.seconds):
        self.counter +=1
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # pylint: disable=no-member ???
        except CvBridgeError as e:
            print(e)
        
        self.img_array.append(cv_image)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
    else:
        self.height, self.width, self.layers = self.img_array[-1].shape
        self.size = (self.width, self.height)

        self.hora = time.localtime()
        self.hora_string = time.strftime("%H:%M:%S", self.hora)
        self.hora_string = self.hora_string.replace(':','_')
    
        out = cv2.VideoWriter(self.PATH_DATA+'video_{}.avi'.format(self.hora_string),cv2.VideoWriter_fourcc(*'MJPG'), 10, self.size)
 
        for i in range(len(self.img_array)):
            out.write(self.img_array[i])
        out.release()
        reason = "Termino la grabacion, numero de frames: {}".format(self.counter)
        print(reason)
        rospy.signal_shutdown(reason)


def main(args):
  if len(args) > 1:
    segundos = int(args[1])
    image_converter(segundos)
  else:
    image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)