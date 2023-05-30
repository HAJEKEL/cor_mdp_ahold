#!/usr/bin/env python3

from gtts import gTTS
import speech_recognition as sr
import rospy
import os
import actionlib
import argparse
from pathlib import Path

from voice_tools import speak

from chatgpt_response import ChatGPTAssistant, ClientResponse
import voice_requests.msg



"""
Action server for a customer interaction with the chatGPT chatbot.
"""
	

class CustomerInteractionServer(object):
    _result = voice_requests.msg.CustomerInteractionResult()


    def __init__(self, name: str, template: Path, max_turns: int = 10):


        self._template = template.read_text()
        self._max_turns = max_turns
        # setup speech recognition
        self._recognizer = sr.Recognizer()
        self._mic = sr.Microphone(device_index=None)
        with self._mic as source:
            self._recognizer.adjust_for_ambient_noise(source)

        self._rate = rospy.Rate(20)
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, 
                                                voice_requests.msg.CustomerInteractionAction, 
                                                execute_cb=self.as_cb, 
                                                auto_start=False)
        self._as.start()

    def user_response(self):
        rospy.loginfo("Listening for user input...")
        mic = sr.Microphone(device_index=None)
        with self._mic as source:
            audio = self._recognizer.listen(source, phrase_time_limit=5)

        try:
            rospy.loginfo("Google Speech Recognition thinks you said : \"" + self._recognizer.recognize_google(audio) + "\"")
            return self._recognizer.recognize_google(audio)
        except sr.UnknownValueError:
            rospy.loginfo("Google Speech Recognition could not understand audio")
            return False
        except sr.RequestError as e:
            rospy.loginfo("Could not request results from Google Speech Recognition service; {0}".format(e))
            return False


    def as_cb(self, goal):
        """
        Callback for action server.
        """
        rospy.loginfo("Starting customer interaction...")
        chatbot = ChatGPTAssistant(self._template)

        # Get initial response from chatbot
        response = chatbot.get_response()
        rospy.loginfo("Chatbot: " + response)
        speak("Hello there, what can I help you with")

        # Start the interaction
        i = 0
        request_done = False
        while i < self._max_turns and not request_done:
            prompt = self.user_response()

            try:
                response = chatbot.get_response(prompt)
            except:
                speak("Sorry, I couldnt respond to that")
                continue

            speak(response.response)
            i += 1

            if response.request_done:
                request_done = True
                # set the result
                self._result.wanted_product = response.wanted_product
                self._result.picking_assistance = response.picking_assistance
                rospy.loginfo(f"Customer wants the product: {self._result.wanted_product} and requires assistance: {self._result.picking_assistance}")
                # publish the result
                self._as.set_succeeded(self._result)
                return

            if self._as.is_preempt_requested():
                rospy.loginfo("Preempted customer interaction")
                self._as.set_preempted()
                return

        # If we get here, the interaction timed out
        rospy.loginfo("Customer interaction timed out")
        self._as.set_aborted()
        return
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Customer interaction server')
    parser.add_argument('--template', type=Path, default=Path(__file__).parent.parent / "config" / "chatgpt_primer.txt", help='Path to template file')

    rospy.init_node('customer_interaction_server')
    CustomerInteractionServer(rospy.get_name(), parser.parse_args().template, max_turns=10)
    rospy.spin()


        













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
