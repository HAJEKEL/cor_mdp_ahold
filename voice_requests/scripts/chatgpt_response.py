import openai
# Marijn's API key
openai.api_key ='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'

import rospy
from pathlib import Path
primer = Path('chatgpt_primer.txt').read_text()

class ClientResponse:
    def __init__(self, text: str):
        response = text.strip('][').split('; ')
        if len(response) != 4:
            raise ValueError("Response does not have the correct format")
        print(response)

        request_done = response[0].split('=')
        self.request_done = request_done[1]
        wanted_product = response[1].split('=')
        self.wanted_product = wanted_product[1]
        picking_assistance = response[2].split('=')
        self.picking_assistance = picking_assistance[1]
        response = response[3].split('=')
        self.response = str(response[1])

    def __str__(self):
        return f"Request done: {self.request_done}\nWanted product: {self.wanted_product}\nPicking assistance: {self.picking_assistance}\nResponse: {self.response}"



class ChatGPTAssistant:
    def __init__(self, model="gpt-3.5-turbo"):
        self.model = model
        self.messages = []
        available_products = "milk, peach tea, mint tea, chocolate" # change this to a query to the parameter server for products later
        self.messages.append({"role": "user", "content": primer.format(available_products)})
        

    def get_response(self, prompt=None):
        if prompt is not None:
            self.messages.append({"role": "user", "content": prompt})
        response = openai.ChatCompletion.create(
            model=self.model,
            messages=self.messages,
            temperature=0,
            )
        self.messages.append(response.choices[0].message)
        return response.choices[0].message["content"]
    


