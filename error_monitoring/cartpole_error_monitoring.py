import autogenu
import matplotlib.pyplot as plt
from sympy import sin, cos, sqrt

# Constants
nx = 4  # Number of states
nu = 1  # Number of control inputs
ocp_name = "generated_code"  # Name of the optimal control problem
Tf = 2.0  # Time horizon for the optimal control problem
alpha = 0.0  # Soft horizon parameter
sampling_time = 0.001  # Sampling time for the discrete-time optimal control problem
N = 100  # Number of discretization intervals
finite_difference_epsilon = 1.0e-08  # Epsilon for finite difference approximations
zeta = 1000  # Penalty parameter
kmax = 5  # Maximum number of iterations for the semi-smooth Newton method
initial_time = 0  # Initial time for the simulation
initial_state = [0, 0, 0, 0]  # Initial state for the simulation
simulation_length = 10  # Length of the simulation in seconds
solution_initial_guess = [
    0.01
]  # Initial guess for the optimal control problem solution
tolerance = 1.0e-06  # Tolerance for the optimal control problem solution
max_iterations = (
    50  # Maximum number of iterations for the optimal control problem solution
)

# Define the AutoGenU object
auto_gen_u = autogenu.AutoGenU(
    ocp_name, nx, nu
)  # Create an AutoGenU object with the specified problem name, number of states, and number of control inputs

# Define variables
t = auto_gen_u.define_t()  # Define time variable
x = auto_gen_u.define_x()  # Define state variables
u = auto_gen_u.define_u()  # Define control input variables
m_c, m_p, l, g = auto_gen_u.define_scalar_vars(
    "m_c", "m_p", "l", "g"
)  # Define scalar variables for the cartpole system (mass of cart, mass of pendulum, length of pendulum arm, and gravity)
q = auto_gen_u.define_array_var(
    "q", nx
)  # Define array variable for the state cost weights
q_terminal = auto_gen_u.define_array_var(
    "q_terminal", nx
)  # Define array variable for the terminal state cost weights
x_ref = auto_gen_u.define_array_var(
    "x_ref", nx
)  # Define array variable for the state reference
r = auto_gen_u.define_array_var(
    "r", nu
)  # Define array variable for the control input cost weights


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
L = sum(q[i] * (x[i] - x_ref[i]) ** 2 for i in range(nx)) / 2 + (r[0] * u[0] ** 2) / 2

# Define the terminal cost
phi = sum(q_terminal[i] * (x[i] - x_ref[i]) ** 2 for i in range(nx)) / 2

# Set functions
auto_gen_u.set_functions(f, C, h, L, phi)

# Set additional parameters
auto_gen_u.add_control_input_bounds(uindex=0, umin=-15.0, umax=15.0, dummy_weight=0.1)
auto_gen_u.set_scalar_vars(["m_c", 2], ["m_p", 0.2], ["l", 0.5], ["g", 9.80665])
auto_gen_u.set_array_var("q", [2.5, 10, 0.01, 0.01])
auto_gen_u.set_array_var("r", [1])
auto_gen_u.set_array_var("q_terminal", [2.5, 10, 0.01, 0.01])
auto_gen_u.set_array_var("x_ref", [0, "M_PI", 0, 00])

# Generate OCP definition
simplification = False
common_subexpression_elimination = True
auto_gen_u.generate_ocp_definition(simplification, common_subexpression_elimination)

# # Set NLP type
nlp_type = autogenu.NLPType.MultipleShooting
auto_gen_u.set_nlp_type(nlp_type)

# Set horizon parameters
auto_gen_u.set_horizon_params(Tf, alpha)

# Set solver parameters
auto_gen_u.set_solver_params(sampling_time, N, finite_difference_epsilon, zeta, kmax)

# Set initialization parameters
auto_gen_u.set_initialization_params(solution_initial_guess, tolerance, max_iterations)

# Set simulation parameters
auto_gen_u.set_simulation_params(initial_time, initial_state, simulation_length)

# Set simulation parameters
auto_gen_u.set_ocp_dir("error_monitoring")

# Generate main function and CMakeLists
auto_gen_u.generate_main()
auto_gen_u.generate_cmake()

# Build and run simulation
generator = "Auto"
vectorize = False
remove_build_dir = False
auto_gen_u.git_submodule_update()
auto_gen_u.build_main(
    generator=generator, vectorize=vectorize, remove_build_dir=remove_build_dir
)
auto_gen_u.run_simulation()

# Plot results
plotter = autogenu.Plotter(
    log_dir=auto_gen_u.get_ocp_log_dir(), log_name=auto_gen_u.get_ocp_name()
)
plotter.set_scales(2, 5, 2)
plotter.save()

# Generate animation
anim = autogenu.CartPole(
    log_dir=auto_gen_u.get_ocp_log_dir(), log_name=auto_gen_u.get_ocp_name()
)
anim.set_skip_frames(10)
anim.generate_animation()

# Generate Python bindings
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

# Generate and open documentation
autogenu.generate_docs()
# autogenu.open_docs()
