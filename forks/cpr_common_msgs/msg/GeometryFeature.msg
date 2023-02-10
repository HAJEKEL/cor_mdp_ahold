uint8 POINT   = 1
uint8 LINE    = 2
uint8 POLYGON = 3
 
string id
uint8  type  # point/line/polygon
 
# one point for POINT, >=2 points for LINE, >=3 points for POLYGON
geometry_msgs/Point[] points
 
Attribute[] attributes
uint8[]     data
