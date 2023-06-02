# Order Package

## Overview

The order package is designed to process product orders received from supermarket customers, connect to a database for order validation, and generate a list of items required for fulfilling the order.

### Features
- Order Processing: The package provides functionalities to receive customer orders via a popup window and initiates the order handling process.
- Database Integration: It integrates with a backend database system to retrieve necessary information, such as april tag number, product location and orientation. The order_handler node puts the database on the ROS parameter server.
- Order Feasibility Check: The package performs feasibility checks on the order, ensuring that all requested products fit within the designated packaging constraints.
- Item List Generation: Upon successful order validation, the package generates a list of items required for fulfilling the order, including their quantities. The order manager node extracts the corrosponding information from the ROS server.

## Usage

Launch the order package with the following command:
bash
```
roslaunch order_package order_handler.py
```

Publish customer orders to the /name_list topic using the provided ROS message format. Ensure that the order message includes all necessary information such as the correct names and quantities.

The package will process the received order, connect to the backend database for order validation, and perform feasibility checks.

If the order is feasible, the package will generate an item list message on the /name_list topic, containing the required items, their quantities, and locations.

## Contact

Contributions to the order package are welcome! If you find any issues or have suggestions for improvements, please open an issue in the issue tracker. For question please mail s.c.j.vijverberg@student.tudelf.nl.

We hope you find this ROS package helpful in handling product orders effectively!
