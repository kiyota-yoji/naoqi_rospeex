#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import ros packages
try:
    import roslib; roslib.load_manifest('rospeex')
except:
    pass

import rospy, numpy, array, audioop, collections, math, wave, tempfile, os
from naoqi_msgs.msg import AudioBuffer
from rospeex_if import ROSpeexInterface

NAOQI_AUDIO_TOPIC = '/naoqi_microphone/audio_raw'

class NaoqiSR():
    def __init__(self, timeout = None):
        rospy.init_node('naoqi_sr')
        self.sub = rospy.Subscriber(NAOQI_AUDIO_TOPIC, AudioBuffer, self.audio_cb)
        # parameters for getting WAV data
        self.energy_threshold = 1000
        self.pause_threshold = 0.8
        self.quiet_duration = 0.5
        self.timeout = timeout
        self._listening = False
        self._frame_id = 0

        # parameters for speech recognition
        self.lang = rospy.get_param('~lang')
        self.engine = rospy.get_param('~engine')

        self._sr_interface = ROSpeexInterface()
        self._sr_interface.init()
        self._sr_interface.register_sr_response(self._sr_response)
        rospy.sleep(0.5)

    def _begin(self, data):
        self._frames = collections.deque()
        assert self.pause_threshold >= self.quiet_duration >= 0
        chunk = len(data.data) / 4
        self._seconds_per_buffer = (chunk + 0.0) / data.frequency
        self._pause_buffer_count = int(math.ceil(self.pause_threshold / self._seconds_per_buffer)) # number of buffers of quiet audio before the phrase is complete
        self._quiet_buffer_count = int(math.ceil(self.quiet_duration / self._seconds_per_buffer)) # maximum number of buffers of quiet audio to retain before and after
        self._elapsed_time = 0
        self._listening = True
        self._speaking = False

    def _output_frames(self, frame_data):
        f = wave.open('output_%04d.wav' % self._frame_id, 'w')
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(16000)
        f.setcomptype('NONE', 'not compressed')
        f.writeframes(frame_data)
        f.close()
        self._frame_id += 1

    def _speech_recognition(self, frame_data):
        # make temporary WAV file
        tmp = tempfile.mkstemp()
        f = wave.open(tmp[1], 'wb')
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(16000)
        f.setcomptype('NONE', 'not compressed')
        f.writeframes(frame_data)
        f.close()

        # send to rospeex
        voice_data = None
        with open(tmp[1], 'rb') as fr:
            voice_data = fr.read()
        print "lang=%s, engine=%s" % (self.lang, self.engine)
        self._sr_interface.recognize(voice_data, self.lang, self.engine)
        os.remove(tmp[1])
    
    def audio_cb(self, data):
        # This contains (in the 'data' field): int16[] data
        #rospy.loginfo("Callback received!")
        
        if not self._listening:
            self._begin(data)

        self._elapsed_time += self._seconds_per_buffer
        #print 'elasped_time: ' + repr(self._elapsed_time)
        if self.timeout and self._elapsed_time > self.timeout:
            pass

        buffer = array.array('h', data.data[::4]).tostring()

        self._frames.append(buffer)

        energy = audioop.rms(buffer, 2)

        if self._speaking:
            if energy > self.energy_threshold:
                self._pause_count = 0
            else:
                self._pause_count += 1
            if self._pause_count > self._pause_buffer_count: # end of the phrase
                # obtain frame data
                for i in range(self._quiet_buffer_count, self._pause_count):
                    self._frames.pop() # remove extra quiet frames at the end
                frame_data = b"".join(list(self._frames))
                print "frame_data: %d, energy = %d" % (len(frame_data), audioop.rms(buffer, 2))
                self._listening = False
                self._speaking = False
                #self._output_frames(frame_data)
                self._speech_recognition(frame_data)
                
        else:
            if energy > self.energy_threshold:
                self._speaking = True
                self._pause_count = 0
            else:
                if len(self._frames) > self._quiet_buffer_count: # ensure we only keep the needed amount of quiet buffers
                    self._frames.popleft()


    def _sr_response(self, message):
        """
        speech synthesis response callback

        @param message: message class
        @type  message: str
        """
        print message


if __name__ == "__main__":
    sr = NaoqiSR()
    rospy.spin()

    
