import numpy as np
import open3d as o3d
import numpy.linalg as LA
import time


start=time.time()

pcd=o3d.io.read_point_cloud("onlypipe_transed_mini.ply")
o3d.geometry.estimate_normals(pcd,search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=0.5,max_nn=10000))
#法線ベクトルを視点方向に揃える
o3d.orient_normals_towards_camera_location(
    pcd,camera_location = np.array([0., 0., 1000.], dtype="float64"))

#o3d.visualization.draw_geometries([pcd])

pcd_tree = o3d.geometry.KDTreeFlann(pcd)


point=np.asarray(pcd.points)
vector=np.asarray(pcd.normals)

lamda3M=np.zeros((len(point),2))

for query_id in range(len(point)):
#for query_id in range(1):
        #query_idは点の番号
        #何mまわりの点からテンソルを計算するか
        d=0.4
        #kは近傍点の数
        [k, idx, dist] = pcd_tree.search_radius_vector_3d(pcd.points[query_id], d)
        sigma=np.zeros((3,3))
        for indexN in range(k):
                #indexNは回りの点の番号(N)
                N=idx[indexN]
                normalV=vector[N,:]
                trans=normalV.reshape(1,len(normalV))
                trans=trans.transpose()
                seki=np.dot(trans,np.matrix(normalV))
                sigma=sigma+seki
        
        tensor=sigma/k
        #print(k)
        [koyuuti,koyuuV]=LA.eig(tensor)
        lamda3=np.amin(np.abs(koyuuti))
        #print(lamda3)
        lamda3M[query_id,0]=lamda3
        if lamda3>0.00095:
                #エルボは１
                lamda3M[query_id,1]=1
        else:
                lamda3M[query_id,1]=0

M=np.block([point,vector])
M=np.block([M,lamda3M])
np.savetxt('point_Nvec_lamda3_0.4m.csv',M,delimiter=',')

#時間計測
elapsed_time = time.time() - start   
print ("演算時間:{0}".format(elapsed_time) + "[sec]")

                






#o3d.visualization.draw_geometries([pcd])
#print(np.asarray(pcd.points[0]))
#print(pcd.normals[0])


# point=np.asarray(pcd.points)
# vector=np.asarray(pcd.normals)
# print(point.shape)
# print(vector.shape)


# M=np.block([point,vector])
# np.savetxt('point_Nvec.csv',M,delimiter=',')

