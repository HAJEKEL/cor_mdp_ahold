import math
import rospy
from tabulate import tabulate
from std_msgs.msg import Int32MultiArray

output_coordinates = []
output_orientations = []
output_list = []

#parameters
box_cap = 40*30*30/2

print("Inventory:")
print("Name, size[cm^3], position[x,y,z][m], orientation[angle][rad]")
table = [["Hagelslag_melk",7*7*15,[0, -1, 0],0],
         ["Hagelslag_puur",7*7*15,[0, 0, 0],0],
         ["Tea_mango",7*7*15,[0, 1, 0],0],
         ["Tea_bosvruchten",5*5*10,[0, 2, 0],0]]
print(tabulate(table, showindex="always"))

def Create_order():
    for product in table:
        name = str(product[0])
        print('Enter amount of '+name+ ":")
        amount = input()
        product.append(int(amount))
    print(tabulate(table))
    return
    #print("Correct?(y/n)")
   # z = input()  
    #if z == "y":
        #    print('Thanks')
    #     return
    # else:
    #     print('Oops, try again')
    #     Create_order()`

def Check_cap():
    global n_boxes
    order_size = 0
    for product in table:
        order_size = order_size + (product[4]*product[1])
    #order_size = int(x)*hagelslag_size+ int(y)*tea_size
    n_boxes = math.ceil(order_size/box_cap)
    print("This order fits in "+ str(n_boxes)+ " box(es).")

def Planner():
    for product in table:
        for item in range(product[4]):
            output_list.append(product[2][0])
            output_list.append(product[2][1])
            output_list.append(product[2][2])
            output_list.append(product[3])
            output_coordinates.append(product[2])
            output_orientations.append(product[3])
            #print("Move to position "+ product[0])
            #publisch location hagelslag
            #Wait for a message: product picked
            #print("Product picked")
            product[4] = product[4]-1 
    
Create_order()
Check_cap()
Planner()
print(output_list)
print(output_coordinates)
print(output_orientations)

#Publisher
if __name__ == "__main__":
    rospy.init_node('order_handler', anonymous=True)

    pub = rospy.Publisher('order_list', Int32MultiArray, queue_size=10)
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        
        # Create the message
        # format output_list[x,y,z,angle]
        msg = Int32MultiArray(data = output_list)
        
        # Validate data before publishing
        if all(isinstance(x, int) for x in msg.data):
            #msg.data = output_coordinates
            pub.publish(msg)
        else:
            rospy.logerr("Invalid data format. Expected an array of integers.")

        # Sleep to maintain the publishing rate
        rate.sleep()

rospy.spin()