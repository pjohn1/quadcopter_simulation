import numpy as np

res = 0.5
max_height = 12.0
x_bounds = (-14.2,13.8)
y_bounds = (-10.7,19.6)
# x_bounds = (-res,res)
# y_bounds = (-res,res)
# max_height = 2.0

with open('/mnt/c/Desktop/quadcopter_simulation/quad_model/meshes/scan3_parsed.pcd','r') as f:
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
    if tuple(coord) not in seen_cords:
        coords = np.vstack((coords,coord))
        seen_cords.add(tuple(coord))
    else:
        continue

coords = coords[np.lexsort( (coords[:,2],coords[:,1],coords[:,0]) )]

clen = len(coords)

open_space = np.empty((0,3))

row = 0
for x,y,z in coords:
    if ( x > x_bounds[0] and x < x_bounds[1] ) and ( y > y_bounds[0] and y < y_bounds[1]):
        if row > 1000 and row%1000 == 0:
            print(f'{row}/{clen}\n')
        mask = (coords[:, 0] == x) & (coords[:, 1] == y) & (coords[:,2] != z)
        z_vals = coords[mask]
        if len(z_vals) == 0:
            all_zs = np.arange(z,max_height,res)
        else:
            z_vals = z_vals[:,2]
            all_zs = np.arange(z,min(z_vals),res)
        
        new_points = np.column_stack((np.full(len(all_zs), x), np.full(len(all_zs), y), all_zs))
        open_space = np.vstack((open_space,new_points))
    row+=1


np.savetxt("grid.txt", open_space, fmt="%.6f", delimiter=" ")
    