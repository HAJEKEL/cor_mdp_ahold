#!/usr/bin/env python3

import speech_recognition as sr
import time
from gtts import gTTS
import os
import rospy


def speak(text):
    tts = gTTS(text=text, lang='en-uk')
    tts.save("voice.mp3")
    os.system("mpg321 voice.mp3")







































