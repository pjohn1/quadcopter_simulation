import numpy as np
pcd_file = "scan3.pcd"
sf = 4
lines = ""
for line in open(pcd_file,'r').readlines():
    try:
        ls = line.split(' ')
        float(ls[0])
        p = sf*np.array([ls[0],ls[1],ls[2][:-1]]).astype(float)
        if p[2] > 12:
            continue
        line = f"{p[0]} {p[1]} {p[2]}\n"
        lines += line
    except ValueError:
        lines += line
        continue

with open(pcd_file,'w') as f:
    f.write(lines)