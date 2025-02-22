import numpy as np

res = 1.0
min_height = 0.17
max_height = 25.0
x_bounds = (-1000,1000)
y_bounds = (-1000,1000)

OBSTACLES = [
    (26.1,30.5,-7.1,-2.9,0.1,5.4),
    (19.3,25.9,-6.42,-3.56,0.1,9.0),
    (14.9,17.9,-6.34,-4.4,0.1,6.5),
    (9.9,13.9,-5.57,-3.6,0.1,8.9),
    (6,7.6,-0.34,2,0.17,1.5),
    (-0.7,8.5,2,4.3,0.17,2.9),
    (-1.3,.83,-0.34,2,0.17,1.5),
    (-9.42,-6.27,-3.45,-1.49,0.1,6.4),
    (-15.8,-11.7,-2.76,0.3,0.1,9.9),
    (-21.9,-17.7,-3.9,-1,0.1,7.5),
    (-19.3,-14.5,2.5,5.6,0.1,4),
    (-18.7,-13.7,2.5,5.6,0.1,10),
    (-18,-16,2.5,5.6,0.1,15.2),
    (-17.2,-16.6,2.5,5.6,0.1,19.5),
    (-13.3,-2.78,2.02,6.79,0.1,11),
    (16.5,20.1,4.0,7.65,0.1,10.6),
    (24.4,30.1,4.0,7.65,0.1,10.6),
    (23.2,30.3,12.5,19.5,0.1,12.6),
    (14.8,22.5,12.5,19.5,0.1,12.6),
    (1.7,7.25,9.8,15.3,0.3,3.8),
    (-7.54,-1.7,9.47,15.9,0.1,11.2),
    (-15.8,6.7,10,7.2,0.1,9.2),
    (-20.1,-10.7,16.8,18.8,0.1,13.4),
    (-21.8,-14.1,23.8,29.8,0.1,18.5),
    (-8.2,-2.2,23.5,30,0.1,11.3),
    (2.57,13,21.9,30.7,0.1,13.5),
    (17.4,27.7,22.7,29.7,0.1,10.1)
] # (x_min,x_max,y_min,y_max,z_min,z_max)

def check_coord(coord):
    return not all(
        [coord[0] > i[0] and coord[0] < i[1] and coord[1] > i[2] and coord[1] < i[3]
         and coord[2] > i[4] and coord[2] < i[5] for i in OBSTACLES]
    )

def check_xy(coord):
    #returns false if coord is not in an obstacle bound
    for i in OBSTACLES:
        if coord[0] >= i[0] and coord[0] <= i[1] and coord[1] >= i[2] and coord[1] <= i[3]:
            return i
    return False

with open('/mnt/c/Desktop/quadcopter_simulation/quad_model/meshes/city.pcd','r') as f:
    pcd_lines = f.readlines()

count=0
for i in pcd_lines:
    if i.split(' ')[0] == 'DATA':
        count+=1
        break
    count+=1

rows = len(pcd_lines) - count

# take lowest z value at each position and essentially stack every 0.1m above it until it hits
# a new z value or max height
'''
If i sort by x value, should go from bottom left to top right
'''


# coords = np.empty((rows,3))
coords = np.empty((0,3))
seen_cords = set()
for i in range(count,rows):
    ls = pcd_lines[i].split(' ')
    coord = np.array((ls[0],ls[1],ls[2][:-1])).astype(float)
    for i in range(3):
        if abs(coord[i]) < 1e-6:
            coord[i] = abs(coord[i])

    if tuple(coord) not in seen_cords and coord[2] > -1e-6:
        coords = np.vstack((coords,coord))
        seen_cords.add(tuple(coord))
    else:
        continue

coords = coords[np.lexsort( (coords[:,2],coords[:,1],coords[:,0]) )]

clen = len(coords)

open_space = np.empty((0,3))

row = 0
for x,y,z in coords:
    if row > 1000 and row%1000 == 0:
        print(f'{row}/{clen}\n')

    z_min = z
    coord_check = (x,y,z)
    obstacle = check_xy(coord_check)
    if obstacle is not False:
        z_min = obstacle[-1] #start at the top of the obstacle

    if z_min < min_height:
        z_min = min_height

    mask = (coords[:, 0] == x) & (coords[:, 1] == y) & (coords[:,2] > z_min)
    #get all z_vals that are above minimum z
    z_vals = coords[mask]

    if len(z_vals) == 0:
        all_zs = np.arange(z_min,max_height,res)
    else:
        z_vals = z_vals[:,2]
        all_zs = np.arange(z_min,min(z_vals),res)
        #only go up to the lowest z val above (happens in case of an overhead obstacle)
    
    # print(z_min)
    new_points = np.column_stack((np.full(len(all_zs), x), np.full(len(all_zs), y), all_zs))
    open_space = np.vstack((open_space,new_points))
    row+=1


np.savetxt("grid.txt", open_space, fmt="%.6f", delimiter=" ")
    