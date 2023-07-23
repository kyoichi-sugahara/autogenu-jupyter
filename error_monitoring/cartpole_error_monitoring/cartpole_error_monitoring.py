import autogenu
import subprocess
import os
import json
import matplotlib.pyplot as plt
from sympy import sin, cos, sqrt

import sys
import pdb

# print(f"{sys._getframe().f_lineno} {__file__}")
    # try:
    #     # Autogenuコード
    #     auto_gen_u.derive_jacobian_matrix(derive_jacobian_matrix)
    # except Exception as e:
    #     # 例外発生時に実行されるコード
    #     exc_type, exc_value, exc_traceback = sys.exc_info()
    #     print("*** Error during code generation:")
    #     print(exc_value)  # エラーメッセージを表示
    #     pdb.post_mortem()  # pdb デバッガーを起動する



def main():
    script_location = os.path.abspath(__file__)
    script_dir = os.path.dirname(script_location)
    # Load the configuration from a JSON file
    with open(script_dir+'/config/config_20230723.json', 'r') as f:
        config = json.load(f)

    # Create and configure the AutoGenU object
    auto_gen_u = create_autogenu_object(config)
    
    if config.get('set_ocp_dir', True):
        # Set the directory for generating optimal control problem code. 
        auto_gen_u.set_ocp_dir("error_monitoring")

    if config.get('set_parameters', True):
        # Set horizon, solver, initialization, and simulation parameters
        set_parameters(auto_gen_u, config)

    if config.get('generate_ocp_definition', True):
        # Generate OCP definition
        generate_ocp_definition(auto_gen_u)

    if config.get('set_nlp_type', True):
        # Set NLP type
        set_nlp_type(auto_gen_u)

    if config.get('derive_symbolic_F', True):
        # Derive Jacobian matrix
        derive_symbolic_F(auto_gen_u)

    if config.get('build_and_run_simulation', True):
        # Set OCP directory, generate main function and CMakeLists, and build and run the simulation
        build_and_run_simulation(auto_gen_u)

    if config.get('plot_results', True):
        # Plot results
        plot_results(auto_gen_u)

    if config.get('generate_animation', True):
        # Generate animation
        generate_animation(auto_gen_u)

    if config.get('generate_and_install_python_bindings', True):
        # Generate and install Python bindings
        generate_and_install_python_bindings(auto_gen_u)

    if config.get('generate_documentation', True):
        # Generate and open documentation
        generate_documentation()


def create_autogenu_object(config):
    auto_gen_u = autogenu.AutoGenU(config["ocp_name"], config["nx"], config["nu"])

    # Define variables
    t = auto_gen_u.define_t()  # Define time variable
    x = auto_gen_u.define_x()  # Define state variables
    u = auto_gen_u.define_u()  # Define control input variables
    m_c, m_p, l, g = auto_gen_u.define_scalar_vars("m_c", "m_p", "l", "g")
    q = auto_gen_u.define_array_var("q", config["nx"])
    q_terminal = auto_gen_u.define_array_var("q_terminal", config["nx"])
    x_ref = auto_gen_u.define_array_var("x_ref", config["nx"])
    r = auto_gen_u.define_array_var("r", config["nu"])

    # Define the state equation
    f = [
        x[2],
        x[3],
        (u[0] + m_p * sin(x[1]) * (l * x[1] * x[1] + g * cos(x[1])))
        / (m_c + m_p * sin(x[1]) * sin(x[1])),
        (
            -u[0] * cos(x[1])
            - m_p * l * x[1] * x[1] * cos(x[1]) * sin(x[1])
            - (m_c + m_p) * g * sin(x[1])
        )
        / (l * (m_c + m_p * sin(x[1]) * sin(x[1]))),
    ]

    # Define the constraints
    C = []
    h = []

    # Define the stage cost
    L = (
        sum(q[i] * (x[i] - x_ref[i]) ** 2 for i in range(config["nx"])) / 2
        + (r[0] * u[0] ** 2) / 2
    )

    # Define the terminal cost
    phi = sum(q_terminal[i] * (x[i] - x_ref[i]) ** 2 for i in range(config["nx"])) / 2

    # Set functions
    auto_gen_u.set_functions(f, C, h, L, phi)

    return auto_gen_u


def derive_symbolic_F(auto_gen_u):
    derive_symbolic_F = True
    auto_gen_u.derive_symbolic_F(derive_symbolic_F)


def generate_ocp_definition(auto_gen_u):
    simplification = False
    common_subexpression_elimination = True
    auto_gen_u.generate_ocp_definition(simplification, common_subexpression_elimination)


def set_nlp_type(auto_gen_u):
    nlp_type = autogenu.NLPType.SingleShooting
    auto_gen_u.set_nlp_type(nlp_type)


def set_parameters(auto_gen_u, config):
    for var in config["scalar_vars"]:
        auto_gen_u.set_scalar_vars([var["name"], var["value"]])

    for var in config["array_vars"]:
        auto_gen_u.set_array_var(var["name"], var["values"])

    auto_gen_u.set_horizon_params(config["Tf"], config["alpha"])
    auto_gen_u.set_solver_params(
        config["sampling_time"],
        config["N"],
        config["finite_difference_epsilon"],
        config["zeta"],
        config["kmax"],
    )
    auto_gen_u.set_initialization_params(
        config["solution_initial_guess"], config["tolerance"], config["max_iterations"]
    )
    auto_gen_u.set_simulation_params(
        config["initial_time"], config["initial_state"], config["simulation_length"]
    )


def build_and_run_simulation(auto_gen_u):
    auto_gen_u.set_ocp_dir("error_monitoring")
    auto_gen_u.generate_main()
    auto_gen_u.generate_cmake()

    generator = "Auto"
    vectorize = False
    remove_build_dir = False
    auto_gen_u.git_submodule_update()
    auto_gen_u.build_main(
        generator=generator, vectorize=vectorize, remove_build_dir=remove_build_dir
    )
    auto_gen_u.run_simulation()


def plot_results(auto_gen_u):
    plotter = autogenu.Plotter(
        log_dir=auto_gen_u.get_ocp_log_dir(), log_name=auto_gen_u.get_ocp_name()
    )
    plotter.set_scales(2, 5, 2)
    plotter.save()


def generate_animation(auto_gen_u):
    anim = autogenu.CartPole(
        log_dir=auto_gen_u.get_ocp_log_dir(), log_name=auto_gen_u.get_ocp_name()
    )
    anim.set_skip_frames(10)
    anim.generate_animation()


def generate_and_install_python_bindings(auto_gen_u):
    generator = "Auto"
    vectorize = False
    remove_build_dir = False
    auto_gen_u.generate_python_bindings()
    auto_gen_u.git_submodule_update()
    auto_gen_u.build_python_interface(
        generator=generator, vectorize=vectorize, remove_build_dir=remove_build_dir
    )

    # Install Python interface
    auto_gen_u.install_python_interface(install_prefix=None)


def generate_documentation():
    autogenu.generate_docs()
    # autogenu.open_docs()


if __name__ == "__main__":
    main()
