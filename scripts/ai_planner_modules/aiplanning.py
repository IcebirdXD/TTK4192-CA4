import subprocess
import os

import subprocess
import os

class GraphPlan:
    def __init__(self, domain_file, problem_file):
        self.domain_file = domain_file
        self.problem_file = problem_file
        self.plan_file = "/home/andreas/catkin_ws/src/temporal-planning-main/temporal-planning/tmp_sas_plan.1"
        self.cwd = "/home/andreas/catkin_ws/src/temporal-planning-main/temporal-planning"
        self.python27_path = "/home/andreas/catkin_ws/src/temporal-planning-main/venv/bin/python2.7"

    def graphPlan(self):
        print("Running STP planner")

        # Run the planner directly with Python 2.7 from the venv
        result = subprocess.run([
        "python2.7", "bin/plan.py", "stp-2",
        self.domain_file, self.problem_file
        ], cwd=self.cwd)

        if not os.path.exists(self.plan_file):
            print("Plan file not found.")
            return []

        with open(self.plan_file, "r") as f:
            lines = [line.strip() for line in f if line.strip() and not line.startswith(";")]

        if not lines:
            print("Plan file is empty.")
            return []

        print("Plan generated. Parsing...")

        actions = []

        for line in lines:
            if not line.startswith("(") and ":" in line:
        # Example: 0.000: ( move_robot turtlebot0 waypoint0 waypoint1 d01 ) [3.0898]
                line = line.split(": ", 1)[-1]  # remove timestamp
                line = line.split(")")[0] + ")"  # strip the time at the end

            parts = line.strip("()").split()
            if not parts:
                continue

            action_name = parts[0]

            if action_name == "move_robot" and len(parts) >= 4:
                target = parts[3]
                actions.append(['move_robot', target])

            elif action_name == "check_seals_valve_picture_eo" and len(parts) >= 5:
                valve = parts[4]
                actions.append(['check_seals_valve_picture_eo', valve])

            elif action_name == "take_pump_picture_ir" and len(parts) >= 5:
                pump = parts[4]
                actions.append(['check_pump_picture_ir', pump])

            elif action_name == "charge_battery":
                actions.append(['charge_battery', None])

            else:
                print("⚠️ Skipping malformed or unknown line:", parts)



        print("Parsed plan actions:", actions)
        return actions




domain_file = "/home/andreas/catkin_ws/src/temporal-planning-main/temporal-planning/domains/ttk4192/domain/PDDL_domain_1.pddl"
problem_file = "/home/andreas/catkin_ws/src/temporal-planning-main/temporal-planning/domains/ttk4192/problem/PDDL_problem_1.pddl"

gp = GraphPlan(domain_file, problem_file)
plan = gp.graphPlan()

print("\n--- PLAN OUTPUT ---")
for i, action in enumerate(plan):
    print(f"{i+1}: {action}")