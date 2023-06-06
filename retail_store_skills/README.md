# Retail Store Skills

This repository contains the action servers used by albert to pick and place products, and to drive around.


## Example

Make sure you first activate the singularity environment and source your workspace for every new terminal instance.

To test a pick skill first launch the simulation:

```bash
roslaunch albert_gazebo albert_gazebo_navigation.launch
```

In another terminal window launch the skills:

```bash
roslaunch retail_store_skills load_skills.launch
```

Wait until moveit is fully initialized in the simulation.
There are action servers for picking the product, presenting the product and placing it in the basket.

In order to pick a specific product you need to check the box "TF" in the display section in  Rviz.
If you do not see the TF box in the display section you must add it by clicking Add -> RVIZ -> TF.

Then you will see it detects the products their local axis by their april tags.
If you still do not see the tags make sure the endpoint of the robotarm is pointed towards the shelves.

Then you can see which frames from the april tags it is detecting.
By going in the display section to TF -> Frames you can see some of them are called "tag_" and with a number.

Pick a frame of the detected tags that you want to pick, for example: tag_18

In yet another terminal window, run the following to execute a pick action:
```bash
rosrun retail_store_skills pick_client.py 18
```
For presenting the product to a customer type in the terminal:
```bash
rosrun retail_store_skills present_client.py 
```
For placing the product in the basket type in the terminal:
```bash
rosrun retail_store_skills place_basket_client.py
```





