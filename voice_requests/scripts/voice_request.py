#!/usr/bin/env python3

import speech_recognition as sr
import time
import threading
from gtts import gTTS
import os

from chatgpt_response import ChatGPTAssistant, ClientResponse



def listen_hello_albert(recognizer, audio):
    print("Listening for \"hello albert\"...")
    try:
        if "albert" in recognizer.recognize_google(audio).lower():
            customer_interaction()

    except sr.UnknownValueError:
        print("hello_alber recognizer could not understand audio")
        return False
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))
        return False

def wait_for_hello_albert():
    print("Listening for \"hello albert\"...")
    mic = sr.Microphone(device_index=None)
    with mic as source:
        try:
            audio = recognizer.listen(source, phrase_time_limit=5)
            if "albert" in recognizer.recognize_google(audio).lower():
                return True
        except sr.UnknownValueError:
            print("hello_alber recognizer could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Google Speech Recognition service; {0}".format(e))


def user_response():
    print("Listening for user input...")
    mic = sr.Microphone(device_index=None)
    with mic as source:
        audio = recognizer.listen(source, phrase_time_limit=5)
    try:
        print("Google Speech Recognition thinks you said : \"" + recognizer.recognize_google(audio) + "\"")
        return recognizer.recognize_google(audio)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand audio")
        return False
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition service; {0}".format(e))
        return False

def customer_interaction(max_turns=10):
    chatbot = ChatGPTAssistant()
    i = 0
    response = chatbot.get_response()
    print("Chatbot: " + response)
    speak(response)

    while i < max_turns:
        prompt = user_response()
        try:
            res = chatbot.get_response(prompt)
        except:
            speak("Sorry, I didn't understand that.")
            continue
        try:
            response = ClientResponse(res)
        except ValueError:
            speak("Sorry, I didn't understand that.")
            continue


        speak(response.response)
        if response.request_done == "True":
            break
        i += 1



def speak(text):
    tts = gTTS(text=text, lang='en-uk')
    tts.save("voice.mp3")
    os.system("mpg321 voice.mp3")
 

if __name__ == "__main__":
    chatbot = ChatGPTAssistant()

    recognizer = sr.Recognizer()
    mic = sr.Microphone(device_index=None)

    with mic as source:
        recognizer.adjust_for_ambient_noise(source)  # we only need to calibrate once, before we start listening

    # while True:
    #     if wait_for_hello_albert():
    #         customer_interaction()  

    # listen for "hello albert" and then start a customer interaction
    stop_listening = recognizer.listen_in_background(mic, listen_hello_albert)

    # do other things on the main thread
    while True: time.sleep(0.1)





































