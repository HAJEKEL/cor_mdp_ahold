# this file is to write a typedb schema to a database
from typedb.client import *
from TypedbKB import TypedbKB
from helper_scripts.problem_generator import write_problem_pddl
from helper_scripts.python_plan_output import print_plan
from helper_scripts.solve_pddl import solve_pddl
from datetime import datetime, timedelta
import argparse

# Parse arguments
parser = argparse.ArgumentParser()
parser.add_argument("--rosplan_ws", type=str, default="../../../../project_melodic_ws/")

args = parser.parse_args()


# Store the current date in yyyy-mm-dd format
# current_date = datetime.today().strftime('%Y-%m-%d')
current_date = "2023-04-6"

# Function to increment the current date by one day
def increment_date():
    global current_date
    date = datetime.strptime(current_date, '%Y-%m-%d')
    new_date = date + timedelta(days=1)
    current_date = new_date.strftime('%Y-%m-%d')


#################################################################
###################### Create the database ######################
#################################################################

db = "KRR_project"

KB = TypedbKB(db)
KB.write_schema("schema.tql", "data.tql")

# add rules for where each type of product should be stored
KB.add_product_destination("milk", "shelf1")
KB.add_product_destination("yogurt", "shelf1")
KB.add_product_destination("meat", "table1")
KB.add_product_destination("cheese", "table1")
KB.add_product_destination("hagelslag", "shelf0")
KB.add_product_destination("coffee", "shelf0")
KB.add_product_destination("chocolate", "table0")

# insert products from csv file
KB.add_products_csv("krr_products.csv")

print("completed initial DB")


#################################################################
########################### Simulation ##########################
#################################################################

success = True # For this simulation we assume that the generated PDDL problems are solved
# Execute restocking until no products are left to be stocked
def work():
    while True:
        try:
            # Attempt to query for max priority products
            products, priority_level = KB.get_max_priority_products()
            # Give neat output with resulting plan
            print_plan(products, priority_level)
            # Automated problem.pddl file generator
            write_problem_pddl(products, "wp0")
            # Solve PDDL problem
            solve_pddl(problem_filename="problem.pddl", 
                       domain_filename="domain_pick_place.pddl",
                       rosplan_ws=args.rosplan_ws) # accepts the user-defined path to the rosplan workspace
            if success:
                # For each product, update the location in the KB
                for product in products:
                    KB.update_product_location(product["name"], product["destination"], quiet=True)
        except:
            break

# Simulate restocking
while (current_date != "2023-04-11"): # 2023-05-02 lets all items expire
    print(f"------------------ {current_date} ------------------")
    # Add new products to KB on 2023-04-08
    if (current_date == "2023-04-08"):
        KB.add_product("yogurt1", "yogurt", 1.20, "stock_table", "2023-04-09")
        KB.add_product("cheese1", "cheese", 5.80, "stock_table", "2023-04-11")
        KB.add_product("hagelslag4", "hagelslag", 2.50, "stock_table", "2023-05-10")
    # Execute restocking
    work()
    # Continue to the next day
    increment_date()
    KB.set_date(current_date)

    





