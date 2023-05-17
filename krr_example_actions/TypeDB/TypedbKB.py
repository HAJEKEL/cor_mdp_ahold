from typedb.client import *
import csv

class TypedbKB:
    def __init__(self, db_name):
        self.db_name = db_name
        self.client = TypeDB.core_client("localhost:1729")
        if self.client.databases().contains(db_name):
            print("creating a fresh new TypeDB database")
            self.client.databases().get(db_name).delete()
        self.client.databases().create(db_name)
        self.session = self.client.session(self.db_name, SessionType.DATA)
        self.options = TypeDBOptions.core()
        self.options.infer = True


    """
    write the schema and (optinally) data from a file to the database
    """
    def write_schema(self, schema_file, data_file):
        with self.client.session(self.db_name, SessionType.SCHEMA) as schema_session:
            with schema_session.transaction(TransactionType.WRITE) as write_transaction:
                with open(schema_file, "r") as schema_file:
                    schema = schema_file.read()
                write_transaction.query().define(str(schema))
                write_transaction.commit()
                print("wrote the schema from: " + schema_file.name)

        if data_file:
            with self.session.transaction(TransactionType.WRITE) as write_transaction:
                with open(data_file, "r") as data_file:
                    data = data_file.read()
                write_transaction.query().insert(str(data))
                write_transaction.commit()
                print("wrote the data from: " + data_file.name)


    """
    add a rule to the knowledge base,
    which states the storage destination of a product type
    """
    def add_product_destination(self, product_type, storage_name): 
        query = f"define rule {product_type}_to_{storage_name}: \
                when {{$product isa {product_type}; \
                $storage isa storage_space, has name \"{storage_name}\"; \
                $product has expiry_date $expiry_date; \
                $date has current_date $current_date; \
                $expiry_date > $current_date; \
                }} then {{ \
                (subject: $product, location: $storage) isa destination; }};"

        with self.client.session(self.db_name, SessionType.SCHEMA) as schema_session:
            with schema_session.transaction(TransactionType.WRITE) as write_transaction:
                write_transaction.query().define(str(query))
                write_transaction.commit()
        print(f"added rule: {product_type}_to_{storage_name}")



    def insert_transaction(self, query):
        with self.session.transaction(TransactionType.WRITE) as write_transaction:
            write_transaction.query().insert(str(query))
            write_transaction.commit()

    def update_transaction(self, query):
        with self.session.transaction(TransactionType.WRITE) as write_transaction:
            write_transaction.query().update(str(query))
            write_transaction.commit()

    def read_transaction(self, query, type="match"):
        with self.session.transaction(TransactionType.READ) as read_transaction:
            if type == "match":
                answer_iterator = read_transaction.query().match(query)
            elif type == "aggregate":
                answer_iterator = read_transaction.query().match_aggregate(query)
            elif type == "group":
                answer_iterator = read_transaction.query().match_group(query)
            elif type == "group_aggregate":
                answer_iterator = read_transaction.query().match_group_aggregate(query)
                owners, numerics = [],[]
                for ans in answer_iterator:
                    owners.append(ans.owner().get_value())
                    numerics.append(ans.numeric().as_int())
                return owners, numerics

            return answer_iterator

    """
    for adding a product instance
    """
    def add_product(self, product_name, product_type, price, current_loc, expiry_date):
        product_definition = f"insert ${product_name} isa {product_type}, has name \"{product_name}\", has price {price}, has expiry_date {expiry_date};"

        product_location = f"match $product isa {product_type}, has name \"{product_name}\"; \
                $location isa storage_space, has name \"{current_loc}\"; \
                insert (subject: $product, location: $location) isa current_location; "

        self.insert_transaction(product_definition)
        self.insert_transaction(product_location)
        print(f"added product {product_name} of type {product_type}")
        self.update_storage_capacity()


    """
    import a csv of products into the KB
    """
    def add_products_csv(self, csv_file):
        with open(csv_file, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                self.add_product(row["name"], row["type"], row["price"], row["current_location"], row["expiry_date"])


    def update_storage_capacity(self):
        query = f"match $storage_space isa storage_space, has name $n; \
                (subject: $food, location: $storage_space) isa current_location; \
                get $food, $n; group $n; count;"

        # retrieve table names and their current capacity
        storages, capacities = self.read_transaction(query, type="group_aggregate")

        # update the capacities
        for storage, capacity in zip(storages, capacities):
            query = f"match $storage_space isa storage_space, has name \"{storage}\"; \
                    $storage_space has current_capacity $c; \
                    delete $c isa current_capacity; \
                    insert $storage_space has current_capacity {capacity};"
            self.update_transaction(query)


    # change item location (and update strorage capacities)
    def update_product_location(self, food_name, new_location, quiet=False):
        query = f"match $food isa food, has name \"{food_name}\"; \
                $new_location isa storage_space, has name \"{new_location}\"; \
                $loc (subject: $food, location: $prev_location) isa current_location; \
                delete $loc (location: $prev_location); \
                insert $loc (location: $new_location);"
        self.update_transaction(query)
        if not quiet:
            print(f"updated {food_name} location to {new_location}")
        self.update_storage_capacity()


    def set_date(self, date):
        query = f"match $date isa date, has current_date $old_date; \
                delete $date has $old_date; \
                insert $date has current_date {date};"
        self.update_transaction(query)

    """
    retun a dict of all of the prducts within the highest available priority level
    """
    def get_max_priority_products(self):
        max_priority_query = "match $product isa food, has priority_level $p; get $p; min $p;"

        with self.session.transaction(TransactionType.READ, self.options) as read_transaction:
            answer = read_transaction.query().match_aggregate(max_priority_query).get()
            if answer.is_int():
                max_priority = answer.as_int()
            else:
                return

        query = f"match $product isa food, has priority_level {max_priority}, \
                has name $n, has expiry_date $e, has price $p; \
                (subject: $product, location: $loc) isa current_location; \
                (subject: $product, location: $dest) isa destination; \
                $loc has name $loc_name; \
                $dest has name $dest_name; \
                (waypoint: $wp_loc, subject: $loc) isa has_waypoint; \
                (waypoint: $wp_dest, subject: $dest) isa has_waypoint; \
                $wp_loc has name $wp_loc_name; \
                $wp_dest has name $wp_dest_name; \
                get $n, $loc_name, $dest_name, $wp_loc_name, $wp_dest_name, $e; sort $e;"

        with self.session.transaction(TransactionType.READ, self.options) as read_transaction:
            answer_iterator = read_transaction.query().match(query)
            products = []
            for ans in answer_iterator:
                product = {}
                product["name"] = ans.get("n").get_value()
                product["current_loc"] = ans.get("loc_name").get_value()
                product["current_wp"] = ans.get("wp_loc_name").get_value()
                product["destination"] = ans.get("dest_name").get_value()
                product["destination_wp"] = ans.get("wp_dest_name").get_value()
                products.append(product)
        return products, max_priority




    def get_waypoints(self):
        query = "match $wp isa waypoint, \
                has name $n,\
                has lin_x $x,\
                has lin_y $y,\
                has lin_z $z,\
                has rot_x $rx,\
                has rot_y $ry,\
                has rot_z $rz;\
                get $n, $x, $y, $z, $rx, $ry, $rz;"

        with self.session.transaction(TransactionType.READ) as read_transaction:
            answer_iterator = read_transaction.query().match(query)
            waypoints = []
            for ans in answer_iterator:
                waypoint = {}
                waypoint["name"] = ans.map().get("n").get_value()
                waypoint["x"] = ans.map().get("x").get_value()
                waypoint["y"] = ans.map().get("y").get_value()
                waypoint["z"] = ans.map().get("z").get_value()
                waypoint["rx"] = ans.map().get("rx").get_value()
                waypoint["ry"] = ans.map().get("ry").get_value()
                waypoint["rz"] = ans.map().get("rz").get_value()
                waypoints.append(waypoint)
            print(waypoints)
            return waypoints


    

                     




     








