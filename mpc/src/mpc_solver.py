from casadi import SX, vertcat, sin, cos, inv
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import numpy as np
import os

m = SX(0.08)
d = 98.0/1000
I = np.zeros((3,3))  # Define as symbolic 3x3 matrix
I[0,0] = 0.00679
I[1,1] = 0.00679
I[2,2] = 0.01313
g= SX(9.81)


NS=4
NU=1

model_vals = SX(NS,1)

# Define pendulum model
def create_pendulum_model():
    model = AcadosModel()
    model.name = "quadcopter"

    # Define states
    x_sym = SX.sym("x")
    vx = SX.sym("vx")
    pitch = SX.sym("pitch")  # Angle
    wy = SX.sym("wy")  # Angular velocity
    x = vertcat(x_sym, pitch, vx, wy)

    # Control input (torque)
    ty = SX.sym("ty")
    # fz_b = SX.sym("fz_b")
    # u = vertcat(ty,fz_b)
    fz_b = m*g*SX(cos(pitch)) #required body force to maintain hover
    # System dynamics
    x_dot_sym = vx
    vx_dot = SX(1/m) * SX(fz_b) * SX(sin(pitch))
    pitch_dot = wy
    wy_dot = SX(inv(I)[1,1]) * ty

    print("Type of x_dot_sym:", type(x_dot_sym))
    print("Type of vx_dot:", type(vx_dot))
    print("Type of pitch_dot:", type(pitch_dot))
    print("Type of wy_dot:", type(wy_dot))

    # Define system dynamics
    # x_dot = vertcat(x_dot_sym,vx_dot,pitch_dot,wy_dot)
    x_dot = vertcat(vx_dot,SX.sym("temp1"),SX.sym("temp2"),SX.sym("temp3"))


    model.x = x
    model.u = vertcat(ty)
    model.xdot = x_dot

    model.f_expl_expr = x_dot  # Explicit dynamics
    model.f_impl_expr = x_dot - SX.sym("xdot", NS)  # Implicit dynamics
    
    return model

# Create and configure NMPC problem
def create_ocp():
    ocp = AcadosOcp()
    ocp.model = create_pendulum_model()
    
    # Horizon length
    N = 10
    T = 0.5  # Total time horizon

    # Set dimensions
    ocp.dims.N = N
    ocp.solver_options.tf = T

    # Cost function weights
    Q = np.eye(NS)  # State cost weights
    R = np.eye(NU)  # Control input weight

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W = np.block([[Q, np.zeros((NS, NU))], [np.zeros((NU, NS)), R]])
    ocp.cost.W_e = Q

    ocp.cost.Vx = np.eye(NS)
    ocp.cost.Vu = np.eye(NU,NS)#np.eye(NU,NS+NU,k=NS)  # Control input matrix
    ocp.cost.Vx_e = np.eye(NS)

    # Constraints
    ocp.constraints.lbu = np.array([-2.0])  # Min torque
    ocp.constraints.ubu = np.array([2.0])   # Max torque
    ocp.constraints.idxbu = np.array([0])

    # Solver settings
    ocp.solver_options.qp_solver = "FULL_CONDENSING_QPOASES"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"

    return ocp

# Generate solver
if __name__ == "__main__":
    ocp = create_ocp()
    json_file = "pendulum_ocp.json"
    print("NS (Number of states):", NS)
    print("NU (Number of control inputs):", NU)
    print("Cost matrix W shape:", ocp.cost.W.shape)
    print("Vx shape:", ocp.cost.Vx.shape)
    print("Vu shape:", ocp.cost.Vu.shape)
    print("Constraints idxbu:", ocp.constraints.idxbu)

    # Export the problem
    ocp_solver = AcadosOcpSolver(ocp, json_file=json_file)

    # Generate C code
    os.system("acados_codegen " + json_file)
