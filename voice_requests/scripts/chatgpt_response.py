import openai
# Marijn's API key
openai.api_key ='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'
from order_package.msg import OrderRequest

import rospy
from pathlib import Path

class ClientResponse:
    def __init__(self, text: str):
        response = text.strip('][').split('; ')
        if len(response) != 4:
            raise ValueError("Response does not have the correct format")
        print(response)

        request_done = response[0].split('=')
        self.request_done = request_done[1] == "True"
        wanted_product = response[1].split('=')
        self.wanted_product = wanted_product[1]
        picking_assistance = response[2].split('=')
        self.picking_assistance = picking_assistance[1] == "True"
        response = response[3].split('=')
        self.response = str(response[1])

    def __str__(self):
        return f"Request done: {self.request_done}\nWanted product: {self.wanted_product}\nPicking assistance: {self.picking_assistance}\nResponse: {self.response}"



class ChatGPTAssistant:
    def __init__(self, template: str, model="gpt-4"):
        self.model = model
        self.messages = []
        self.template = template

        # get the available products from the parameter server
        available_products = rospy.get_param("/products")
        available_products = ", ".join(available_products.keys())
        
        # add the available products to the template
        self.messages.append({"role": "user", "content": self.template.format(available_products)})


    def get_response(self, prompt=None, n_tries: int = 3) -> ClientResponse:
        if prompt is not None:
            self.messages.append({"role": "user", "content": prompt})

        for _ in range(n_tries):
            res = openai.ChatCompletion.create(
                model=self.model,
                messages=self.messages,
                temperature=0
            )

            if prompt is None:
                return res.choices[0].message["content"]

            try:
                response = ClientResponse(res.choices[0].message["content"])
                self.messages.append(res.choices[0].message)
                return response
            except ValueError:
                rospy.logerr(f"Response attempt {_ + 1} does not have the correct format")

        # If we get here, the response does not have the correct format
        raise ValueError("Response does not have the correct format")



    


