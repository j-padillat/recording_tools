#!/usr/bin/env python3
from __future__ import print_function

import sounddevice as sd
from scipy.io.wavfile import write
import threading

import rospkg
import sys
import rospy
import cv2
from sensor_msgs.msg import Image, CompressedImage
from speech_utilities_msgs.srv import talk_speech_srv, saveAudio_srv
from cv_bridge import CvBridge, CvBridgeError
import time
import rostopic
import rosservice
import numpy as np

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


class multimedia_recorder:

  def __init__(self, segundos=8):
    self.seconds = segundos
    self.RosOn = False
    try:
      availableTopics = rostopic.get_topic_list()
      self.availableServices = rosservice.get_service_list()
      self.RosOn = True
    except:
      self.RosOn = False
    self.bridge = CvBridge()
    if self.RosOn and '/record_audio/save_audio_srv' in self.availableServices:
      self.image_sub = rospy.Subscriber("/robot_toolkit_node/camera/front/image_raw/compressed", CompressedImage, self.callback_pepper)
      print(consoleFormatter.format("Waiting for save audio service", "WARNING"))
      rospy.wait_for_service('/record_audio/save_audio_srv')
      print(consoleFormatter.format("save audio service connected!", "OKGREEN"))
      self.saveAudioServiceClient = rospy.ServiceProxy('/record_audio/save_audio_srv', saveAudio_srv)
      self.talkSpeechServiceClient = rospy.ServiceProxy('record_audio/talk_speech_srv', talk_speech_srv)
      self.talkSpeechServiceClient('You can talk to me now', 1, 'English')
      self.threadAudio = threading.Thread(target=self.audio_local_recording, args=(self.seconds,))
    else:
      self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback_local)
      self.threadAudio = threading.Thread(target=self.audio_local_recording, args=(self.seconds,))
    self.tiempo_inicial = time.time()
    self.img_array = []
    self.counter = 0
    self.PATH_RECORDING_TOOLS = rospkg.RosPack().get_path('recording_tools')
    self.PATH_VIDEOS = self.PATH_RECORDING_TOOLS+'/resources/videos/'
    self.PATH_AUDIOS = self.PATH_RECORDING_TOOLS+'/resources/audios/'

  def callback_local(self,data):
    self.actual_time = time.time()
    if (self.actual_time - self.tiempo_inicial <= self.seconds):
      ### --- RECORD AUDIO LOCAL
      # if self.counter == 0:
      #   self.threadAudio.start()
      ### ---
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
  
      out = cv2.VideoWriter(self.PATH_VIDEOS+'video_{}.avi'.format(self.hora_string),cv2.VideoWriter_fourcc(*'DIVX'), 10, self.size)

      for i in range(len(self.img_array)):
        out.write(self.img_array[i])
      out.release()
      reason = "Termino la grabacion, numero de frames: {}".format(self.counter)
      print(reason)
      rospy.signal_shutdown(reason)

  def callback_pepper(self,msg):
    self.actual_time = time.time()
    if (self.actual_time - self.tiempo_inicial <= self.seconds):
      ### --- RECORD AUDIO PEPPER
      if self.counter == 0:
        self.threadAudio.start()
      ### ---
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
  
      out = cv2.VideoWriter(self.PATH_VIDEOS+'video.avi',cv2.VideoWriter_fourcc(*'MJPG'), 10, self.size)

      for i in range(len(self.img_array)):
        out.write(self.img_array[i])
      out.release()

  def audio_local_recording(self, seconds):

    if self.RosOn and '/record_audio/save_audio_srv' in self.availableServices:
      self.saveAudioServiceClient(self.seconds)
      reason = "Termino la grabacion, numero de frames: {}".format(self.counter)
      print(reason)
      rospy.signal_shutdown(reason)
    else:
      fs = 44100  # Sample rate
      myrecording = sd.rec(int(seconds * fs), samplerate=fs, channels=1)
      sd.wait()  # Wait until recording is finished
      # For the file name
      self.hora = time.localtime()
      self.hora_string = time.strftime("%H:%M:%S", self.hora)
      self.hora_string = self.hora_string.replace(':','_')

      # Constants
      self.PATH_AUDIO = self.PATH_AUDIOS + 'audio.wav'
      write(self.PATH_AUDIO, fs, myrecording)  # Save as WAV file 



def main(args):
  if len(args) > 1:
    segundos = int(args[1])
    multimedia_recorder(segundos)
  else:
    multimedia_recorder()
  rospy.init_node('multimedia_recorder', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    consoleFormatter=ConsoleFormatter()
    print(consoleFormatter.format(" --- multimedia_recorder node initialized --- ", "OKGREEN"))
    main(sys.argv)