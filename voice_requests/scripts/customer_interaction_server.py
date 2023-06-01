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
        n_fails = 0
        while i < self._max_turns and not request_done:
            prompt = self.user_response()

            try:
                response = chatbot.get_response(prompt)
            except:
                n_fails += 1
                if n_fails > 2:
                    speak("Lets try this again")
                    self._as.set_aborted()
                    return
                speak("Sorry, I didn't understand that.")
                continue


            speak(response.response)
            i += 1

            if response.request_done and response.wanted_product is not None and response.picking_assistance is not None:
                request_done = True
                # set the result
                self._result.success = True
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

    # if available, get the path to the template file from the parameter server
    if rospy.has_param("/gpt_primer_path"):
        template = Path(rospy.get_param("/gpt_primer_path"))
    else:
        template = Path(__file__).parent.parent / "config" / "chatgpt_primer.txt"



    rospy.init_node('customer_interaction_server')
    CustomerInteractionServer(rospy.get_name(), template, max_turns=10)
    rospy.spin()


