from casadi import SX, vertcat, horzcat, sin, cos, nlpsol, sum1

# Define physical constants
m = SX(0.08)  # Mass (kg)
g_ = SX(9.81)  # Gravity (m/s²)
I_yy = SX(0.00679)  # Moment of inertia around y-axis

# Define time horizon
N = 20  # Number of steps in prediction horizon
dt = 0.05  # Time step

# Define goal state
x_goal = SX([1.0, 0.0, 0.0, 0.0])  # Goal: move to x = 1, stabilize pitch

# Define symbolic states and controls for N steps
X = horzcat(*[SX.sym(f"X_{k}", 4) for k in range(N+1)])  # 4 states per timestep
U = horzcat(*[SX.sym(f"U_{k}", 1) for k in range(N)])  # 1 control input per timestep

# Initialize cost function
cost = 0
g = []  # Constraints (dynamics)

# Loop through horizon
for k in range(N):
    x_k = X[:, k]  # Current state at step k
    u_k = U[:, k]  # Control input at step k

    # Compute system dynamics
    x_dot_sym = x_k[1]
    vx_dot =  g_*cos(x_k[2]) * sin(x_k[2])
    pitch_dot = x_k[3]
    wy_dot = u_k[0] / I_yy
    # Discretize with Euler
    # print(x_k,vertcat(x_k[1], vx_dot, x_k[3], wy_dot))
    x_next_k = x_k + dt * vertcat(x_k[1], vx_dot, x_k[3], wy_dot)

    # Add cost: tracking + control effort
    cost += 0.1 * sum1(u_k**2) + 1000 * sum1((x_k - x_goal) ** 2)
    


    # Add constraints for system dynamics
    g.append(X[:, k+1] - x_next_k)

# Flatten constraint list
g = vertcat(*g)

# Flatten state & control for NLP
XU = vertcat(X.reshape((-1,1)), U.reshape((-1,1)))  # Column vector

# Define NLP problem
nlp = {
    "x": XU,  # Optimize full state+control trajectory
    "f": cost,  # Objective function
    "g": g  # Dynamics constraints
}

# Create solver
solver = nlpsol("solver", "ipopt", nlp)

# Initial state
x0 = [0, 0, 0.1, 0]  # Start at rest

# Initial guess for state and control
X0 = x0 * (N+1)
U0 = [0] * N  # Initial control guess
x_init = X0 + U0  # Combine into single list

# Constraint bounds
lbg = [0] * (4 * N)  # Dynamics must be satisfied
ubg = [0] * (4 * N)

# Torque bounds
lbx = [-float('inf')] * (4 * (N+1)) + [-2] * N  # Allow full range of state, min torque -2
ubx = [float('inf')] * (4 * (N+1)) + [2] * N  # Torque max 2

# Solve the NLP
sol = solver(x0=x_init, lbg=lbg, ubg=ubg, lbx=lbx, ubx=ubx)

# Extract optimal state and control sequence
optimal_X = sol["x"][: 4 * (N+1)].full().reshape((4, N+1))  # State trajectory
optimal_U = sol["x"][4 * (N+1) :].full().reshape((1, N))  # Control trajectory

# Print solver results
# print("Solver status:", solver.stats())
print("Optimal state trajectory:", sol["f"])
# print("Optimal control sequence:", optimal_U)
