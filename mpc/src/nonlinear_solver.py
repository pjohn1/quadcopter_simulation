import numpy as np
from scipy.integrate import odeint
from scipy.optimize import minimize, differential_evolution
import time

dt = 0.5

NS = 8
NU = 2

Q = np.eye(NS) + np.array([10]*NS).transpose()#np.array([10,1,1,1,10,1,1,1]).transpose() #pose weights
R = np.eye(NU) + np.array([1e-2]*NU).transpose() #control weights

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
    x,y,roll,pitch,vx,vy,wx,wy = X
    roll = -roll
    tx,ty = args[0]
    m = args[1]
    Ixx,Iyy = args[2]
    Fz = args[3]

    Fzn = getRotationMatrix(roll,pitch,0) @ np.array([0,0,Fz]).transpose() #convert body force to inertial frame

    xdot = vx
    xdotdot = 1/m*Fzn[0]
    ydot = vy
    ydotdot = 1/m*Fzn[1]
    pitchdot = wy
    pitchdotdot = ty/Iyy
    rolldot = wx
    rolldotdot = tx/Ixx

    return [xdot,xdotdot,ydot,ydotdot,pitchdot,pitchdotdot,rolldot,rolldotdot]

def cost_fn(u,*args):
    # u is an NSxNU matrix representing the control horizon
    X = np.array(args[0])
    goal = np.array(args[1])
    N = args[2]
    m = 0.08
    g = 9.81
    Ixx = 0.00679
    Iyy = 0.00679
    num_timesteps = 5

    cost = 0
    costu = 0
    costv = 0
    for i in range(0,N,NU):
        Fz = (getRotationMatrix(X[3],X[4],0) @ np.array([0,0,m*g]).transpose())[2]
        X = np.array(odeint(update_state,X,np.linspace(0,dt,num_timesteps),args=(u[i:i+NU],m,(Ixx,Iyy),Fz))[-1])
        print(X[0],X[1])
        value = np.array(goal-X)
        cost += u[i:i+NU] @ R @ u[i:i+NU].transpose() + value @ Q @ value.transpose()
        costu += u[i:i+NU] @ R @ u[i:i+NU].transpose()
        costv += value @ Q @ value.transpose()
    # print(costu,costv)
    return cost



def main(N,X0,goal):
    u0 = np.array([0]*N*NU)

    # u = minimize(cost_fn,u,args=(X0,goal,N))
    # u = differential_evolution(cost_fn,bounds,args=(X0,goal,N))
    t1 = time.time()
    u = minimize(cost_fn,u0,args=(X0,goal,N),method='Nelder-Mead')
    print(u.success)
    if u.success:
        print(f'operating at {1/(time.time()-t1)} Hz')
        print(f'final control {u.x[0:NU]}, final value{u.fun}')
        return u.x[0:NU],u.fun
    else:
        return None,None


#u is what we are optimizing and X is the output of that optimization
# main(3,[0,2,0,0],[1,0,0,0])/
main(3,[0]*8,[0.5]+[0]*7)