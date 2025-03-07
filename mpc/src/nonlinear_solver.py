import numpy as np
from scipy.integrate import odeint
from scipy.optimize import minimize, differential_evolution
import time

dt = 0.05

Q = np.eye(4) + np.array([1,1,0.1,1]).transpose() #pose weights
R = np.eye(2) + np.array([100,100]).transpose() #control weights

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
    num_timesteps = 5

    cost = 0
    for i in range(N):
        Fz = m*g*np.cos(X[2])
        X = np.array(odeint(update_state,X,np.linspace(0,dt,num_timesteps),args=(u[i],m,Iyy,Fz))[-1])
        value = np.array(goal-X)
        # print(value)
        cost += 100*u[i]**2 + value @ Q @ value.transpose()
    return cost



def main(N,X0,goal):
    u0 = [0.0]*N

    # u = minimize(cost_fn,u,args=(X0,goal,N))
    # u = differential_evolution(cost_fn,bounds,args=(X0,goal,N))
    t1 = time.time()
    u = minimize(cost_fn,u0,args=(X0,goal,N),method='Nelder-Mead')
    print(u.success)
    if u.success:
        print(f'operating at {1/(time.time()-t1)} Hz')
        print(f'final control {u.x[0]}, final value{u.fun}')
        return u.x[0],u.fun
    else:
        return None,None


#u is what we are optimizing and X is the output of that optimization
# main(3,[0,2,0,0],[1,0,0,0])