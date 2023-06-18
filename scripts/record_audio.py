#!/usr/bin/env python3
###################################### IMPORTS #########################################

import rospy
import rospkg
import time
import speech_recognition as sr
import numpy as np
import soundfile as sf

######## MESSAGES ########
from naoqi_bridge_msgs.msg import AudioBuffer
from robot_toolkit_msgs.msg import audio_tools_msg, speech_msg, leds_parameters_msg, misc_tools_msg
from robot_toolkit_msgs.srv import audio_tools_srv, misc_tools_srv
from speech_utilities_msgs.srv import talk_speech_srv, saveAudio_srv

######## SUBMODULES ########
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



class RecordAudio:
    # -----------------------------------------------------INIT--------------------------------------------------------------
    def __init__(self) -> None:
        # Service names
        self.talk_server = rospy.Service('record_audio/talk_speech_srv', talk_speech_srv, self.callback_talk_speech_srv)
        print(consoleFormatter.format('talk_speech_srv on!', 'OKGREEN'))
        self.saveAudio_server = rospy.Service('record_audio/save_audio_srv', saveAudio_srv, self.callback_save_audio_srv)
        print(consoleFormatter.format('save_audio_srv on!', 'OKGREEN'))

        # Subscribers
        self.micSubscriber=rospy.Subscriber("/mic", AudioBuffer, self.audioCallbackSingleChannel)
        
        # Publishers
        self.ledsPublisher = rospy.Publisher('/leds', leds_parameters_msg, queue_size=10)
        self.t2s_msg = speech_msg()
        #self.t2s_msg.language = "English"
        self.t2s_msg.animated = True
        self.speech_pub=rospy.Publisher('/speech', speech_msg, queue_size=10)
        
        # Service speech call
        self.audioMessage = audio_tools_msg()
        self.audioMessage.command = "enable_tts"
        self.audioToolsService(self.audioMessage)

        # For the file name
        self.hora = time.localtime()
        self.hora_string = time.strftime("%H:%M:%S", self.hora)
        self.hora_string = self.hora_string.replace(':','_')

        # Constants
        self.PATH_RECORDING_TOOLS = rospkg.RosPack().get_path('recording_tools')
        self.PATH_AUDIO = self.PATH_RECORDING_TOOLS+'/resources/audios/audio_{}.wav'.format(self.hora_string)
        self.SAMPLE_RATE=16000
        self.audioBuffer = []
        self.ready=False
        self.isTalking = False

        # Attributes
        self.timeRecord = 10
        self.audioMessage = audio_tools_msg()
        self.miscMessage = misc_tools_msg()
        self.ledsMessage = leds_parameters_msg()

        # Misc service
        #misc = rospy.ServiceProxy('/robot_toolkit/misc_tools_srv', misc_tools_srv)
        self.miscMessage.command = "enable_all"
        self.miscToolsService(self.miscMessage)

        self.audioFile = sr.AudioFile(self.PATH_AUDIO)

        #Service Clients    ### NO DEBERIA SER EN ESTE NODO.
        # print(consoleFormatter.format("Waiting for save audio service", "WARNING"))
        # rospy.wait_for_service('record_audio/save_audio_srv')
        # print(consoleFormatter.format("save audio service connected!", "OKGREEN"))
        # self.saveAudioServiceClient = rospy.ServiceProxy('record_audio/save_audio_srv', saveAudio_srv)



# ----------------------------------------------------AUXILIAR FUNCTIONS-------------------------------------------------
    def miscToolsService(self,msg):
        """
        Enables the misc Tools service from the toolkit of the robot.
        """
        print(consoleFormatter.format("Waiting for audio tools service", "WARNING"))
        rospy.wait_for_service('/robot_toolkit/misc_tools_srv')
        try:
            misc = rospy.ServiceProxy('/robot_toolkit/misc_tools_srv', misc_tools_srv)
            miscService = misc(msg)
            print(consoleFormatter.format("Misc tools service connected!", "OKGREEN"))
        except rospy.ServiceException as e:
            print(consoleFormatter.format("Service call failed: %s"%e,"FAIL"))
    
    def audioToolsService(self,msg):
        """
        Enables the audio Tools service from the toolkit of the robot.
        """
        print(consoleFormatter.format("Waiting for audio tools service", "WARNING"))
        rospy.wait_for_service('/robot_toolkit/audio_tools_srv')
        try:
            audio = rospy.ServiceProxy('/robot_toolkit/audio_tools_srv', audio_tools_srv)
            audioService = audio(msg)
            print(consoleFormatter.format("Audio tools service connected!", "OKGREEN"))
        except rospy.ServiceException as e:
            print(consoleFormatter.format("Service call failed: %s"%e,"FAIL"))
    
    def saveRecording(self,fileName, sampleRate):
        """
        Function for save the audio
        Args:
            fileName (string): Name of the audio which will be saved.
        """
        if self.ready:
            audioBuffer1 = np.array(self.audioBuffer)
            audioBuffer1 = audioBuffer1.astype(float)
            audioBuffer1 = np.asarray(audioBuffer1, dtype=np.int16) 
            sf.write(self.PATH_AUDIO,audioBuffer1,sampleRate,closefd=True)
            #with sf.SoundFile(self.PATH_AUDIO, 'r+') as f:
                #f.SoundFile.close()
            #sf.SoundFile.close()
            #saveAudio=sf.write(self.PATH_AUDIO,audioBuffer1,sampleRate)
            #saveAudio.close()

            '''
            if self.checkFileExistance == True:
                remove(self.PATH_AUDIO)
                sf.write(self.PATH_AUDIO,audioBuffer1,sampleRate)
                print("Recording Audio is Finish. Audio File Created")
            elif self.checkFileExistance == False:
                sf.write(self.PATH_AUDIO,audioBuffer1,sampleRate)
                print("Recording Audio is Finish. Audio File Created")
            '''
    
    def ledsColor(self,r,g,b):
        """
        Function for setting the colors of the eyes of the robot.
        Args:
        r,g,b numbers
            r for red
            g for green
            b for blue
        """
        self.ledsMessage.name = "FaceLeds"
        self.ledsMessage.red = r
        self.ledsMessage.green = g
        self.ledsMessage.blue = b
        self.ledsMessage.time = 0
        self.ledsPublisher.publish(self.ledsMessage)  #Inicio(aguamarina), Pepper esta lista para escuchar
        time.sleep(0.5)

    def speak(self,p_text,p_sleep,language):
        """
        Given the text via a string it publish on the topic the message and the robot say the text.
        Args:
            p_text (string): Text that the robot must say.
            p_sleep (float): Sleep for the robot.
            language (string): English, Spanish
        """
        # Method for speech to text purposes
        self.t2s_msg.language = language
        self.t2s_msg.text = p_text
        self.speech_pub.publish(self.t2s_msg)
        time.sleep(p_sleep)



# ----------------------------------------------------SERVICES CALLBACKS-------------------------------------------------
    def audioCallbackSingleChannel(self,data):
        """
        Callback for the microphone subcriber of the robot
        """

        if self.ready==True:
            audio = data.data
            self.audioBuffer = list(self.audioBuffer)
            audio = list(audio)
            for i in range(0, len(audio)):
                self.audioBuffer.append(audio[i])
    
    ###// LLAMADA: rosservice call /speech_utilities/save_audio_srv 5
    ###// PARAMETROS: srv file: int64 seconds\\---\\string answer

    def callback_save_audio_srv(self,req):

        """
        Callback for save_audio_srv service: This service allows to record an audio with the microphones of the robot with a duration of x seconds.
        Args:
            seconds (int64): Indicates the duration for recording the audio.
        Returns:
            Returns an answer (string): Indicates if the audio file was created.
        """

        print("Requested saveAudio service")
        print(consoleFormatter.format("Requested saveAudio service", "WARNING"))

        #print(req.seconds)
        self.ready=True
        self.ledsPublisher
        #self.saveAudioServer
        self.miscMessage.command = "enable_all"
        self.miscToolsService(self.miscMessage) #Enable LEDs only
        self.audioMessage.command = "custom"
        self.audioMessage.frequency = 16000
        self.audioMessage.channels = 3
        self.audioToolsService(self.audioMessage) # Front Channel F = 16KHz
        print(consoleFormatter.format("Speech Recognition is Inactive 1", "WARNING"))
        self.micSubscriber
        #micSubscriber=rospy.Subscriber("/mic", AudioBuffer, self.audioCallbackSingleChannel)
        self.ledsColor(0,255,255)   #Inicio(aguamarina), Pepper esta lista para escuchar
        print(consoleFormatter.format("Recording Audio", "WARNING"))
        self.audioBuffer = []
        time.sleep(req.seconds) # segundos de silencio
        #remove = os.remove(self.PATH_AUDIO)
        #rospy.signal_shutdown(self.saveRecording(self.PATH_AUDIO,self.audioMessage.frequency))
        self.saveRecording(self.PATH_AUDIO,self.audioMessage.frequency)
        print(consoleFormatter.format("Speech Recognition is Inactive 2", "WARNING"))
        print("Speech Recognition is Inactive 2")
        self.ledsColor(255,255,255) #Grabando Audio(azul)
        self.ready=False
        #rospy.signal_shutdown(self.ledsColor(255,255,0))
        #return rospy.signal_shutdown('Audio Saved')
        return 'Audio Saved'

    ### // LLAMADA: rosservice call /speech_utilities/talk_speech_srv 'Buenas buenass hoy amanceimos' 1 'Spanish'
    ### // Service arguments are: [key sleep language]

    def callback_talk_speech_srv(self,req):
        """
        Callback for talk_speech_srv service: This service allows the robot to say the input of the service.
        Args:
            req: key (string) Indicates the phrase that the robot must say
            req: sleep (float32) Indicates the sleep for the robot while speaking
            req: language (string) Indicates the language which Pepper will speak. 
        Returns:
            Returns 'answer' that indicates what Pepper said.
        """
        print(consoleFormatter.format("\nRequested talk service", "WARNING"))
        

        #print(req.key)
        self.audioMessage.command = "enable_tts"

        self.audioToolsService(self.audioMessage)
        #print(req.key,req.sleep,req.language)

        self.speak(req.key,req.sleep,req.language)

        return 'Pepper said' + req.key




# --------------------------------------------------------MAIN-----------------------------------------------------------

if __name__ == '__main__':
    consoleFormatter=ConsoleFormatter()
    rospy.init_node('record_audio')
    speechUtilities = RecordAudio()
    try:
        print(consoleFormatter.format(" --- record_audio node successfully initialized ---","OKGREEN"))
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
