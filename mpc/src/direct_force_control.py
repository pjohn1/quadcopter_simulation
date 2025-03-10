import numpy as np
from scipy.integrate import odeint
from scipy.optimize import minimize, differential_evolution
import time

def getRotationMatrix(roll,pitch,yaw):

    R3 = np.array([[np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]])
                
    R2 = np.array([[np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R1 = np.array([[1, 0, 0],
            [0, np.cos(roll), np.sin(roll)],
            [0, -np.sin(roll), np.cos(roll)]])
    
    return R3 @ R2 @ R1
# x y roll pitch vx vy wx wy

def update_state(X,t,*args):
    # print(args)
    # print(X,t,args)
    # state is x,vx,pitch,pitch_rate
    # args is u,m,I,Fzb
    # u is f0,f1,f2,f3
    x,y,roll,pitch,vx,vy,wx,wy = X
    f0,f1,f2,f3 = args[0]
    m = args[1]
    Ixx,Iyy = args[2]
    Fz = args[3]

    tx = d * (f1 + f2 - f0 - f3)
    ty = d * (f2 + f3 - f0 - f1)

    Fzn = getRotationMatrix(roll,pitch,0) @ np.array([0,0,Fz]).transpose() #convert body force to inertial frame

    xdot = vx
    xdotdot = 1/m*Fzn[0]
    ydot = vy
    ydotdot = 1/m*Fzn[1]
    pitchdot = wy
    pitchdotdot = ty/Iyy
    rolldot = wx
    rolldotdot = tx/Ixx

    return [xdot,ydot,rolldot,pitchdot,xdotdot,ydotdot,rolldotdot,pitchdotdot]

def cost_fn(u,*args):
    # u is an NSxNU matrix representing the control horizon
    X = np.array(args[0])
    goal = np.array(args[1])
    N = args[2]
    m = 0.08
    g = 9.81
    Ixx = 0.00679
    Iyy = 0.00679
    num_timesteps = 2

    cost = 0
    costu = 0
    costv = 0
    for i in range(0,N,NU):
        Fz = (getRotationMatrix(X[3],X[4],0) @ np.array([0,0,m*g]).transpose())[2]
        X = np.array(odeint(update_state,X,np.linspace(0,dt,num_timesteps),args=(u[i:i+NU],m,(Ixx,Iyy),Fz)))[-1]
        # print(X)
        value = np.array(goal-X)
        # print(value)
        cost += u[i:i+NU] @ R @ u[i:i+NU].transpose() + value @ Q @ value.transpose()
        costu += u[i:i+NU] @ R @ u[i:i+NU].transpose()
        costv += value @ Q @ value.transpose()
    # print(costu,costv)
    return cost



def main(N,X0,goal):

    global dt,NS,NU,Q,R,d
    dt = 0.5
    d = 98/1000
    NS = 8
    NU = 4

    Q = np.eye(NS) * np.array([20,20,1,1,10,10,1,1]).transpose() #pose weightsN
    # print(Q)
    R = np.eye(NU) * np.array([1e-2]*NU).transpose() #control weights

    u0 = np.array([0.0]*N*NU)
    # u0 = np.array([0,0,1.0,1.0])
    # cost_fn(u0,X0,goal,N)
    # u = minimize(cost_fn,u0,args=(X0,goal,N),method='Nelder-Mead')
    u = minimize(cost_fn,u0,args=(X0,goal,N),method='L-BFGS-B')
    print(u.success)
    if u.success:
        return *u.x[0:NU],u.fun
    else:
        return None,None,None,None


#u is what we are optimizing and X is the output of that optimization
# main(3,[0,2,0,0],[1,0,0,0
# print(main(3,[0]*8,[1.0]+[0]*7))
# print(update_state())
# print(odeint(update_state,X,np.linspace(0,dt,10),args=(u[i:i+NU],m,(0.00679,0.00679),Fz)))
# print(cost_fn([0.0,0.0,1.0,1.0],))