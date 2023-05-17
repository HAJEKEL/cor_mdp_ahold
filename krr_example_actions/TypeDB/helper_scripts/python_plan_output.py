def print_plan(products, priority_level):
    plan = "##############################################"
    plan += f"\n     MOVING PRODUCTS OF PRIORITY {priority_level}\n"

    for i, product in enumerate(products):
        plan += f"{i+1}.     {product['name']} from {product['current_loc']} to {product['destination']}  \n"

    plan +="##############################################"


    print(plan)
