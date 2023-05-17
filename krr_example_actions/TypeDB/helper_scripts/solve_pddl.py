from jinja2 import Template, Environment, FileSystemLoader
import subprocess

def solve_pddl(problem_path: str = "pddl_files/", problem_filename: str = "problem.pddl",
               domain_path: str = "pddl_files/",
               domain_filename: str = "domain_pick_place.pddl",
               solver_path: str = "src/ROSPlan/rosplan_planning_system/common/bin/popf",
               rosplan_ws: str = "~/project_melodic_ws/"):

    print("Solving PDDL problem")

    solver_path_joined = rosplan_ws+solver_path
    print("Solver path: ", solver_path_joined)

    # Execute bash command to run the POPF solver on our domain+problem files
    subprocess.run([rosplan_ws+solver_path, domain_path+domain_filename, problem_path+problem_filename])


