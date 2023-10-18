# ---
# jupyter:
#   jupytext:
#     formats: ipynb,py:percent
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.11.5
#   kernelspec:
#     display_name: Python 3
#     language: python
#     name: python3
# ---

# %%
import numpy as np
import teaserpp_python as teaser
import re
import pathlib


# %%
def parse_float(num_string):
    hyphen_pos = num_string.find("-")
    if hyphen_pos == -1:
        return float(num_string)
    return float(num_string[hyphen_pos:])


def parse_array(array_string):
    rows = [x.split(" ") for x in array_string.split("\n") if x != ""]
    rows = [[parse_float(x) for x in row] for row in rows]
    return np.array(rows)


def solve_problem(match):
    src = parse_array(first_problem.group(1))
    dest = parse_array(first_problem.group(2))
    print(f"source: {src}")
    print(f"dest: {dest}")

    params = teaser.RobustRegistrationSolver.Params()
    params.estimate_scaling = False
    params.rotation_estimation_algorithm = (
        teaser.RobustRegistrationSolver.ROTATION_ESTIMATION_ALGORITHM.GNC_TLS
    )
    params.cbar2 = 1.0
    params.noise_bound = 1.0e-3
    # params.rotation_gnc_factor = 1.4
    # params.rotation_max_iterations = 100
    # params.rotation_cost_threshold = 1e-12
    print(f"params: {params}")

    solver = teaser.RobustRegistrationSolver(params)
    # solver.solve(src, dest)

    solution = solver.getSolution()
    if solution.valid:
        print(f"solution:", solution)
    else:
        print("solver failed!")


# %%
log_dir = "/home/ubuntu/lcd_results/uh2_office/dn/trial_1/"
log_relpath = "logs/hydra_dsg_builder_incremental_node.INFO"

# %%
log_path = pathlib.Path(log_dir) / log_relpath
with log_path.open("r") as fin:
    contents = fin.read()

# %%
problem_finder = re.compile(
    r"Source: $(.*?)^I1201.*Dest: $(.*?)^I1201", flags=re.DOTALL | re.MULTILINE
)
first_problem = problem_finder.search(contents)

# %%
solve_problem(first_problem)

# %%
