import open3d as o3d

def PLY2OBJ():
    print("Testing IO for meshes ...")
    mesh = o3d.io.read_triangle_mesh("C:/Modeling/extract/2200.ply")
    print(mesh)
    o3d.io.write_triangle_mesh("C:/Modeling/extract/2201.obj",mesh)
if __name__=='__main__':
    PLY2OBJ()