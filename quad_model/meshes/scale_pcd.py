import numpy as np
pcd_file = "city.pcd"
sf = 4
res= 0.5

lines = ""
already_created = set()
for line in open(pcd_file,'r').readlines():
    try:
        ls = line.split(' ')
        float(ls[0])
        p = sf*np.array([ls[0],ls[1],ls[2][:-1]]).astype(float)
        if p[2] > 25:
            continue

        p = np.round(p/res) * res
        if tuple(p) not in already_created:
            line = f"{p[0]} {p[1]} {p[2]}\n"
            lines += line
            already_created.add(tuple(p))
        else:
            continue
    except ValueError:
        lines += line
        continue

with open("city_parsed.pcd",'w') as f:
    f.write(lines)