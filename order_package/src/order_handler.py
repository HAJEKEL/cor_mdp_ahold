import math
import rospy
from tabulate import tabulate

#parameters
box_cap = 40*30*30/2

print("Inventory:")
table = [["Hagelslag",7*7*15,[1,2,3]],["Tea",5*5*10,[1,2,3]]]
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
        order_size = order_size + (product[3]*product[1])
    #order_size = int(x)*hagelslag_size+ int(y)*tea_size
    n_boxes = math.ceil(order_size/box_cap)
    print("This order fits in "+ str(n_boxes)+ " box(es).")

def Planner():
    for product in table:
        for item in range(product[3]):
            print("Move to position "+ product[0])
            #publisch location hagelslag
            #Wait for a message: product picked
            print("Product picked")
            product[3] = product[3]-1 
    
Create_order()
Check_cap()
Planner()
