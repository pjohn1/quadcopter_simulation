import numpy as np
from scipy.integrate import odeint
from scipy.optimize import minimize, differential_evolution

dt = 0.5


def update_state(X,t,*args):
    # print(X,t,args)
    # state is x,vx,pitch,pitch_rate
    # args is u,m,Iyy,Fzb
    ty = args[0]
    m = args[1]
    Iyy = args[2]
    Fz = args[3]

    xdot = X[1]
    xdotdot = 1/m*Fz*np.sin(X[2])
    thetadot = X[3]
    thetadotdot = ty/Iyy

    return [xdot,xdotdot,thetadot,thetadotdot]

def cost_fn(u,*args):
    # u is an Nx1 matrix representing the control horizon
    X = np.array(args[0])
    goal = np.array(args[1])
    N = args[2]
    m = 0.08
    g = 9.81
    Iyy = 0.00679
    num_timesteps = 10

    cost = 0
    for i in range(N):
        Fz = m*g*np.cos(X[2])
        X = odeint(update_state,X,np.linspace(0,dt,num_timesteps),args=(u[i],m,Iyy,Fz))[-1]
        cost += u[i]**2 + np.linalg.norm(goal - X)
    return cost



def main(N,X0,goal):
    u0 = [0.0]*N

    # u = minimize(cost_fn,u,args=(X0,goal,N))
    # u = differential_evolution(cost_fn,bounds,args=(X0,goal,N))
    u = minimize(cost_fn,u0,args=(X0,goal,N),method='Nelder-Mead')
    print(f'final control {u.x[0]}, final value{u.fun}')
    return u.x[0]


#u is what we are optimizing and X is the output of that optimization
