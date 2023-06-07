string product_name             # Name of the product
string location_name           # Name of the store location
geometry_msgs/Pose waypoint     # Pose of the store location waypoint
float64 base_x                  # x position of the base waypoint
float64 base_y                  # y position of the base waypoint
float64 base_yaw                # yaw position of the base waypoint
uint8 request_type              # pick in basket (0), pick for customer (1), show customer location (2), return to base station (3)
float64 order_id                # Unique order ID. If customer order set to 0. If return to base, set to 100
uint32 min_tag_id               # Lowest apriltag in product range
uint32 max_tag_id               # Highest apriltag


