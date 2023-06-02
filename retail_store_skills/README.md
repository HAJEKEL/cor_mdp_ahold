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

In yet another terminal window, wait until moveit is fully initialized in the simulation then run the following to execute a pick action:
```bash
rosrun actionlib_tools axclient.py /pick_server
```
A GUI will pop up in which you can specify which apriltag id you would like to pick. For example id 18 is reachable from the initial position.

There are action servers for picking the product, presenting the product and placing it in the basket.
For picking the product type in the terminal:
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





