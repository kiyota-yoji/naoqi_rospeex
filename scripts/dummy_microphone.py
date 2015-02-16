#!/usr/bin/env python
# -*- coding: utf-8 -*-

### use local microphone as dummy microphone of pepper

import rospy
from naoqi_msgs.msg import AudioBuffer
from speech_recognition import Microphone
import threading
import time

NAOQI_AUDIO_TOPIC = '/naoqi_microphone/audio_raw'

from ctypes import *
import pyaudio
# From alsa-lib Git 3fd4ab9be0db7c7430ebd258f2717a976381715d
# $ grep -rn snd_lib_error_handler_t
# include/error.h:59:typedef void (*snd_lib_error_handler_t)(const char *file, int line, const char *function, int err, const char *fmt, ...) /* __attribute__ ((format (printf, 5, 6))) */;
# Define our error handler type
ERROR_HANDLER_FUNC = CFUNCTYPE(None, c_char_p, c_int, c_char_p, c_int, c_char_p)
def py_error_handler(filename, line, function, err, fmt):
    pass
    #print 'messages are yummy'
c_error_handler = ERROR_HANDLER_FUNC(py_error_handler)
asound = cdll.LoadLibrary('libasound.so.2')
# Set error handler
asound.snd_lib_error_set_handler(c_error_handler)

class DummyMic():

    def __init__(self):
        rospy.init_node('dummy_microphone')
        self.pub = rospy.Publisher(NAOQI_AUDIO_TOPIC, AudioBuffer, queue_size=10)
        self.source = Microphone()
        self.source.CHUNK = 1365
        self.end_flag = False

    def listen(self):
        with self.source as s:
            while not self.end_flag:
                buffer = s.stream.read(self.source.CHUNK)
                if len(buffer) == 0: break # reached end of the stream
                self._publish(buffer, s)

    def close(self):
        self.end_flag = True

    def _publish(self, buffer, s):
        audio_msg = AudioBuffer()

        # Deal with the sound
        mictmp = []
        for i in range (0,len(buffer)/2) :
            mictmp.append(ord(buffer[2*i])+ord(buffer[2*i+1])*256)

        # convert 16 bit samples to signed 16 bits samples
        for i in range (0,len(mictmp)) :
            if mictmp[i]>=32768 :
                mictmp[i]=mictmp[i]-65536

        # assign the same samples to all 4 channels
        mic_data = []
        for i in range(len(mictmp)):
            mic_data = mic_data + [mictmp[i]] * 4
        
        audio_msg.header.stamp = rospy.Time.now()
        audio_msg.frequency = s.RATE
        audio_msg.channelMap = [3,5,0,2]
        audio_msg.data = mic_data
        
        #print len(audio_msg.data)
        self.pub.publish(audio_msg)

class ListenThread(threading.Thread):

    def __init__(self, mic):
        super(ListenThread, self).__init__()
        self.mic = mic

    def run(self):
        self.mic.listen()

    def exit(self):
        self.mic.close()
        time.sleep(1.0)

if __name__ == "__main__":
    mic = DummyMic()
    th = ListenThread(mic)
    th.setDaemon(True)
    th.start()

    rospy.spin()
    th.exit()
