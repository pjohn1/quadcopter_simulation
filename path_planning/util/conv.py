import open3d as o3d

#this file converts stl 3d model to pcd

# Load the mesh
mesh_path = "/mnt/c/Desktop/quadcopter_simulation/quad_model/meshes/city.stl"
mesh = o3d.io.read_triangle_mesh(mesh_path)

# # Ensure the mesh is triangulated
# mesh.compute_triangle_normals()
# mesh.remove_duplicated_vertices()
# mesh.remove_duplicated_triangles()
# mesh.remove_degenerate_triangles()
# mesh.remove_unreferenced_vertices()

# Convert to a point cloud
pcd = mesh.sample_points_uniformly(number_of_points=50000)

# Save as a PCD file
pcd_output_path = "/mnt/c/Desktop/quadcopter_simulation/quad_model/meshes/city.pcd"
o3d.io.write_point_cloud(pcd_output_path, pcd, write_ascii=True)

print(f"Converted and UV-stripped PCD file saved to: {pcd_output_path}")
