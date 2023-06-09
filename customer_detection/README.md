# ROS Package: Customer Detection

This ROS package provides a node called "customer_detection_node" that detects customers in an image using the HOG (Histogram of Oriented Gradients) method. The node subscribes to a color image topic, applies the HOG detector, and publishes a boolean value indicating whether a customer is detected in the frame.

## Dependencies

- ROS (Robot Operating System) - Melodic or newer
- `cv_bridge` Python package
- OpenCV (with Python bindings)

## Installation

1. Clone the package into your ROS workspace's source directory:
```
cd <your_ros_workspace>/src
git clone <package_url>
```


2. Build the package:
```
cd <your_ros_workspace>
catkin_make
```


## Usage

1. Launch the `customer_detection_node`:
```
roslaunch customer_detection customer_detection.launch

```

2. The node subscribes to the `/realsense_435/color/image_raw` topic to receive color images from a camera. Make sure to publish color images on this topic.

3. The node performs customer detection using the HOG method and publishes the detection result as a boolean value on the `/customer_detected` topic.

### Subscribed Topics

- `/realsense_435/color/image_raw` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
- The color image used for customer detection.

### Published Topics

- `/customer_detected` ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))
- A boolean value indicating whether a customer is detected in the frame.

### Parameters

- `~info_message_rate` (float, default: 1)
- The rate at which the node logs information messages (in seconds). Set to 0 to disable information messages.


## Acknowledgments

This package utilizes the following external libraries:

- OpenCV - https://opencv.org/

## Contact

For any issues or suggestions regarding this package, please contact the maintainer:

Maintainer: Henk Jekel

Email: hendrikjekel@gmail.com
