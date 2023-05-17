from jinja2 import Template, Environment, FileSystemLoader


def write_problem_pddl(products: dict, start_waypoint: str, path: str = "pddl_files/", problem_filename: str = "problem.pddl"):
    enviroment = Environment(loader=FileSystemLoader('templates/'))

    waypoints = [p["current_wp"] for p in products] + [p["destination_wp"] for p in products]
    waypoints = list(set(waypoints))

    # Create the problem file
    problem_template = enviroment.get_template('problem_template.txt')
    context = {
        "problem_name": 'pick_place',
        "products": products,
        "waypoints": waypoints,
        "start_wp": start_waypoint,
        }

    with open(path + problem_filename, mode="w", encoding="utf-8") as results:
        results.write(problem_template.render(context))
        print(f"... wrote {problem_filename}")

