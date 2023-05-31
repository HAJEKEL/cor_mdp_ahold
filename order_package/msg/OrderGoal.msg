string product_name             # Name of the product
string location_name           # Name of the store location
geometry_msgs/Pose waypoint     # Pose of the store location waypoint
uint8 request_type              # pick in basket (0), pick for customer (1), show customer location (2), return to base station (3)
uint32 order_id                 # Unique order ID. If customer order set to 0. If return to base, set to 100
uint32[] april_tags             # April tags of the product type


