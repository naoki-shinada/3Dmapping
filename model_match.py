import numpy as np
import open3d as o3d
import numpy.linalg as LA
import time
import random


#RANSACパラメータ
r=0.5
maxitteration=100
threshould=0.01

start=time.time()

for i in range(10):
    pcd=o3d.io.read_point_cloud("cylinderPLY/"+str(i+1)+"cylinder2m.ply")
    o3d.geometry.estimate_normals(pcd,search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.5,max_nn=10000))
    #法線ベクトルを視点方向に揃える
    o3d.orient_normals_towards_camera_location(
        pcd,camera_location = np.array([0., 0., 1000.], dtype="float64"))

    #o3d.visualization.draw_geometries([pcd])
    
    point=np.asarray(pcd.points)
    normal=np.asarray(pcd.normals)

    maxinliner=0
    itteration=0
    #RANSAC
    while itteration<maxitteration:
        sample=random.sample(range(len(point)),2)
        a=sample[0]
        b=sample[1]
        point1=pcd.points[a]-r*pcd.normals[a]
        point2=pcd.points[b]-r*pcd.normals[b]
        jiku=np.array(point1-point2)

        #jiku=np.array([0,1,0])
        #point1=[0,0,0]

        if np.linalg.norm(jiku)<0.01:
            continue

        jiku=jiku/np.linalg.norm(jiku)

        #一つ一つの点を見る
        count=0
        for x in pcd.points:
            v=np.array(x-point1)
            d=(np.linalg.norm(v)**2-np.dot(v,jiku)**2)**0.5
            if r-threshould<=d<=r+threshould:
                count+=1
        
        if maxinliner<count:
            maxinliner=count
            bestP=point1
            bestJIKU=jiku
       
        itteration+=1
    
    M=np.block([bestP,bestJIKU])
    if i==0:
        result=M
    result=np.vstack([result,M])
    np.savetxt('normal_match.csv',result,delimiter=',')
    # print("point")
    # print(bestP)
    # print("軸")
    # print(bestJIKU)
   

    #時間計測
elapsed_time = time.time() - start   
print ("演算時間:{0}".format(elapsed_time) + "[sec]")