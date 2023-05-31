#!/usr/bin/env python3

"""
This function reads the products.cxv file and creates a parameter for each product.
"""

import rospy
import pandas as pd
from pathlib import Path


def main():
    # initialize the node
    rospy.init_node("product_param_loader")

    # read the path from the argument
    product_data_path = rospy.get_param("/product_param_loader/product_data_path")


    # read the csv file
    products = pd.read_csv(product_data_path)
    print("products: ", products)

    # create a parameter for each product
    for index, row in products.iterrows():
        # create a dictionary for the product
        product_dict = {
            "store_location": row["store_location"],
            "min_tag_id": row["min_tag_id"],
            "max_tag_id": row["max_tag_id"],
        }
        rospy.loginfo("Creating parameter for " + row["product_name"] + " at " + row["store_location"])

        # set the parameter
        rospy.set_param("/products/" + row["product_name"], product_dict)


if __name__ == "__main__":
    main()







