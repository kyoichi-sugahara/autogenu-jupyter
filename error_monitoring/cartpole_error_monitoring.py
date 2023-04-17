import autogenu
import matplotlib.pyplot as plt
from sympy import sin, cos, sqrt


def main():
    # Constants
    config = {
        "nx": 4,  # Number of states
        "nu": 1,  # Number of control inputs
        "ocp_name": "generated_code",  # Name of the optimal control problem
        "Tf": 2.0,  # Time horizon for the optimal control problem
        "alpha": 0.0,  # Soft horizon parameter
        "sampling_time": 0.001,  # Sampling time for the discrete-time optimal control problem
        "N": 100,  # Number of discretization intervals
        "finite_difference_epsilon": 1.0e-08,  # Epsilon for finite difference approximations
        "zeta": 1000,  # Penalty parameter
        "kmax": 5,  # Maximum number of iterations for the semi-smooth Newton method
        "initial_time": 0,  # Initial time for the simulation
        "initial_state": [0, 0, 0, 0],  # Initial state for the simulation
        "simulation_length": 10,  # Length of the simulation in seconds
        "solution_initial_guess": [
            0.01
        ],  # Initial guess for the optimal control problem solution
        "tolerance": 1.0e-06,  # Tolerance for the optimal control problem solution
        "max_iterations": 50,  # Maximum number of iterations for the optimal control problem solution
    }

    # Create and configure the AutoGenU object
    auto_gen_u = create_autogenu_object(config)

    # Set horizon, solver, initialization, and simulation parameters
    set_parameters(auto_gen_u, config)

    # Generate OCP definition
    generate_ocp_definition(auto_gen_u)

    # Set NLP type
    set_nlp_type(auto_gen_u)

    # Set OCP directory, generate main function and CMakeLists, and build and run the simulation
    build_and_run_simulation(auto_gen_u)

    # Plot results
    plot_results(auto_gen_u)

    # Generate animation
    generate_animation(auto_gen_u)

    # Generate and install Python bindings
    generate_and_install_python_bindings(auto_gen_u)

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


def generate_ocp_definition(auto_gen_u):
    simplification = False
    common_subexpression_elimination = True
    auto_gen_u.generate_ocp_definition(simplification, common_subexpression_elimination)


def set_nlp_type(auto_gen_u):
    nlp_type = autogenu.NLPType.SingleShooting
    auto_gen_u.set_nlp_type(nlp_type)


def set_parameters(auto_gen_u, config):
    # auto_gen_u.add_control_input_bounds(
    #     uindex=0, umin=-15.0, umax=15.0, dummy_weight=0.1
    # )
    auto_gen_u.set_scalar_vars(["m_c", 2], ["m_p", 0.2], ["l", 0.5], ["g", 9.80665])
    auto_gen_u.set_array_var("q", [2.5, 10, 0.01, 0.01])
    auto_gen_u.set_array_var("r", [1])
    auto_gen_u.set_array_var("q_terminal", [2.5, 10, 0.01, 0.01])
    auto_gen_u.set_array_var("x_ref", [0, "M_PI", 0, 0])

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
