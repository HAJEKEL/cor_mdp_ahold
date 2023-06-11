# ROS Package: Customer Detection

This ROS package provides a node called "customer_detection_node" that detects customers in an image using the HOG (Histogram of Oriented Gradients) method. The node subscribes to a color image topic, applies the HOG detector, and publishes a boolean value indicating whether a customer is detected in the frame.

## Dependencies

- ROS (Robot Operating System) noetic
- `cv_bridge` Python package: This package is required for converting images between ROS messages and OpenCV images. 
- OpenCV (with Python bindings): This library is used to import the HOG detector.


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
