# Order Package

## Overview

The order package is designed to process product orders received from supermarket customers, connect to a database for order validation, and generate a list of items required for fulfilling the order.

### Features
- Order Processing: The package provides functionalities to receive customer orders via a popup window and initiates the order handling process.
- Database Integration: It integrates with a backend database system to retrieve necessary information, such as april tag number, product location and orientations.
- Order Feasibility Check: The package performs feasibility checks on the order, ensuring that all requested products fit within the designated packaging constraints.
- Item List Generation: Upon successful order validation, the package generates a detailed list of items required for fulfilling the order, including their quantities and locations.

## Installation

To use the order package, you need to have a working ROS environment. Follow the ROS installation instructions at http://wiki.ros.org/ROS/Installation if you haven't already set up ROS.

Next, clone this repository into your catkin workspace and build the package using the following commands:

bash
```
cd <path_to_your_catkin_workspace>/src
git clone https://gitlab.tudelft.nl/cor/ro47007/2023/team-20/cor_mdp_ahold/-/new/order_list/order_package.git
cd ..
catkin build
```
## Usage

Launch the Supermarket Order Handling package with the following command:
bash
```
roslaunch order_package order_handler.py
```

Publish customer orders to the /name_list topic using the provided ROS message format. Ensure that the order message includes all necessary information such as customer details, product IDs, and quantities.

The package will process the received order, connect to the backend database for order validation, and perform feasibility checks.

If the order is feasible, the package will generate an item list message on the /name_list topic, containing the required items, their quantities, and locations.

## Contributing

Contributions to the order package are welcome! If you find any issues or have suggestions for improvements, please open an issue in the issue tracker.

We hope you find this ROS package helpful in handling product orders effectively!
