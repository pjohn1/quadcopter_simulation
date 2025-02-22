import octomap
import numpy as np
import pcl

def pointcloud_to_octree(pcd_file, resolution=0.1):
    cloud = pcl.load_XYZ(pcd_file)
    points = np.asarray(cloud.to_array())

    tree = octomap.OcTree(resolution)

    for point in points:
        tree.updateNode(tuple(point), True)  # Mark as occupied

    print("Octree built with", tree.size(), "nodes.")
    return tree

# Example usage
tree = pointcloud_to_octree("input.pcd", 0.1)

# Save to .bt file (optional)
tree.writeBinary("output.bt")
