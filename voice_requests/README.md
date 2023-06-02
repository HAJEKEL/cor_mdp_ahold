# The Albert Voice Control Package

This package implements voice control functionality for the Albert robot to interact with customers in the retail store.
It uses a chatGPT backend coupled with speech recognition and text to speech functionality in order to assist a customer in the store.



## Customer Interaction
Just like with siri or alexa, a customer in the store can start an interaction by saying the trigger word "Albert".
Listening for this word is done by the `request_detector` node, which sets the `request_pending` parameter to true when the trigger word is heard.

The speech interaction with the customer is handled by an action server, called the `customer_interaction_server`. 
During an interaction, the robot will try to figure out which product the customer wants, and whether the customer requires picking assistance. Once the robot knows this information, it will send a request to the `order_node`, which is explained in detail in the [order_package](../order_package/README.md) 



## chatGPT integration
The true power of this package lies in the chatGPT-powered backend. We use the [openai python api](https://platform.openai.com/docs/api-reference) to interact with the chatGPT API.

### Context
Every time a customer interaction takes place, a new chatGPT session is started.  In order for the chatbot to generate meaningful responses that can be parsed into usefull information, we provide it with context.
The products available in the store are gathered from the parameter server and inserted into this context, so that the chatbot will have an understanding of which products are available.

### Parsing
In order to reliably parse chatGPT's responses, we use a small "hack": we tell chatGPT to always answer in the format: 

*[request_done=(bool); wanted_product=(string); picking_assistance=(bool); response=(string)]*.  

This ensures that we always know if an interaction is complete, and which product the customer wants..

### Benefits
Using chatGPT for the backend has several benefits:
- The chatbot understands the customer's intent, even if the voice recognition is not perfect.
- The chatbot knows how to respond to questions like "do you have products needed to make a cake?" or "do you have dairy products?". This means this information does not have to be added manually to the product database.




## Dependencies
The dependencies needed for this package are listed in this repositorie's main [README.md](../README.md) file.


## Running the package

As mentioned above, the chatbot recieves the products available in the store from the parameter server.
The setting of these parameters is done by the [order_package launch file](../order_package/launch/order_handler.launch), so we need to make sure that this is running.

To run the package:

first, start the order package:
```bash
roslaunch order_package order_handler.launch
```

Then, start the voice control package:
```bash
roslaunch voice_requests voice_requests.launch
```

### "Hello Albert"

To check if everything is working as expected, you can try saying "Hello Albert" to the robot.
If everything is working correctly, the robot should respond with "Hello, how can I help you?".


### Customer interaction

To test a customer interaction, run the example client node provided in this package:
```bash
rosrun voice_requests customer_interaction_client.py
```

**note: Depending on your internet connection and load on the chatGPT servers, it might take a little while for the chatbot to respond.**

**note: in the terminal you may see `ALSA lib ...` errors.  These can be safely ignored.**


### Expected output

If everything is working correctly, and the chatbot has understood your request, you should see something like this in the terminal:








 

