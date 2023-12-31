#!/usr/bin/env python3

import speech_recognition as sr
import time
from gtts import gTTS
import rospy
import sounddevice

from voice_tools import speak

"""
This script spawn a node that listens on the laptop microphone to detect a keyword.
When this keyword is detected, it sets a parameter on the parameter server, indicating that a customer request is pending.
The robot can then start a customer interaction whenever it is ready with its current task.
"""

class KeywordDetector:
    def __init__(self, keyword: str, rate: int = 20, request_param: str = "/request_pending"):
        self._keyword = keyword
        self._rate = rospy.Rate(rate)
        self._recognizer = sr.Recognizer()
        self._mic = sr.Microphone(device_index=None)
        with self._mic as source:
            self._recognizer.adjust_for_ambient_noise(source)
        self._request_param = request_param


    def listen(self):
        rospy.loginfo("Listening for keyword...")
        with self._mic as source:
            audio = self._recognizer.listen(source, phrase_time_limit=5)
        try:
            return self._recognizer.recognize_google(audio)
            
        except sr.UnknownValueError:
            return False
        except sr.RequestError as e:
            rospy.loginfo("Could not request results from Google Speech Recognition service; {0}".format(e))
            return False


    def run(self):
        while True:
            if rospy.get_param(self._request_param) == False:
                result = self.listen()
                print(result)
                if self._keyword in str(result).lower():
                    rospy.set_param(self._request_param, True)
                    rospy.loginfo("Keyword detected!")
                    speak("I'll be with you in a moment.")
                    rospy.loginfo("Request pending...")
                

if __name__ == "__main__":
    rospy.init_node("keyword_detector")
    detector = KeywordDetector("albert", rate=5, request_param="/request_pending")
    detector.run()

        


