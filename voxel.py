
import numpy as np
import open3d as o3d
import argparse
import os 

def get_voxel_coo(res,max_point,min_point):
    index=np.arange(res**3)
    x_=(index % res)[:,None]
    y_=((index // res )%res)[:,None]
    z_=(index // (res**2))[:,None]
    query_point=np.concatenate((z_,y_,x_),axis=1).astype(np.float32)
    # print(max_point[0]-min_point[0])
    query_point[:,0]=(query_point[:,0]+0.5)*((max_point[0]-min_point[0])/res)+min_point[0]
    query_point[:,1]=(query_point[:,1]+0.5)*((max_point[1]-min_point[1])/res)+min_point[1]
    query_point[:,2]=(query_point[:,2]+0.5)*((max_point[2]-min_point[2])/res)+min_point[2]
    query_point = o3d.core.Tensor(query_point, dtype=o3d.core.Dtype.Float32)
    return query_point
def voxelization_solid(mesh,res,mode):
    
    
    voxel_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh, 
                                    voxel_size=(mesh.get_max_bound()[0]-mesh.get_min_bound()[0])/res)
    all_voxel = voxel_grid.get_voxels()
    voxel_array = np.zeros((res,res,res), dtype=int)
    for i in range(len(all_voxel)):
        voxel_array[all_voxel[i].grid_index[0]-1,all_voxel[i].grid_index[1]-1,all_voxel[i].grid_index[2]-1] = 1
    if mode=='surface':
        return voxel_array
    query_point = get_voxel_coo(res,mesh.get_max_bound(),mesh.get_min_bound())
    mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh)
    # Create a scene and add the triangle mesh
    scene = o3d.t.geometry.RaycastingScene()
    _ = scene.add_triangles(mesh)  # we do not need the geometry ID for mesh
    
    occupancy = scene.compute_occupancy(query_point)

    voxel=occupancy.numpy().reshape(res,res,res)
    
    # return voxel
    return np.logical_or(voxel, voxel_array).astype(np.int)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description="microstructure voxelization")

    parser.add_argument('--mesh', type=str,default='S-T-0-1-2.obj', help='mesh file.')
    parser.add_argument('--mode', type=str, default='solid',help='surface or solid')
    parser.add_argument('--res', type=int, default=64,help='surface or solid')
    parser.add_argument('--visualization', type=int, default=True,help='If need voxel visualization')
    args=parser.parse_args()
    # knot_mesh = o3d.data.KnotMesh()
    # mesh = o3d.io.read_triangle_mesh(knot_mesh.path)
    # armadillo_mesh = o3d.data.ArmadilloMesh()
    # mesh = o3d.io.read_triangle_mesh(armadillo_mesh.path)
    mesh = o3d.io.read_triangle_mesh(os.path.join('model',args.mesh))
    mesh.compute_vertex_normals()
    voxel=voxelization_solid(mesh,args.res,args.mode)


    if args.visualization:
        with open(os.path.join('voxel-visualization',args.mesh), 'w') as f:
            for i in range (0,args.res):
                for j in range (0,args.res):
                    for k in range (0,args.res):
                        if voxel[i,j,k]:
                            print('v ',i,j,k,file=f)
    voxel = voxel.reshape(-1)
    np.savetxt(os.path.join('voxel',args.mesh+'.csv'),voxel,fmt='%s')
