import numpy as np
import open3d as o3d
import numpy.linalg as LA
import time
import random


#RANSACパラメータ
minR=0.45
maxR=0.55
maxitteration=1000
threshould=0.05

start=time.time()

for i in range(1):
    pcd=o3d.io.read_point_cloud("stright.ply")
    #pcd=o3d.io.read_point_cloud("cylinderPLY/"+str(i+1)+"cylinder2m.ply")
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
        p1=np.array(pcd.points[a])
        p2=np.array(pcd.points[b])
        n1=np.array(pcd.points[a])
        n2=np.array(pcd.points[b])

        r1=(np.dot(p1-p2,n1)-np.dot(np.dot(n1,n2)*(p1-p2),n2))/(1-(np.dot(n1,n2))**2)
        r2=(-np.dot(p1-p2,n2)+np.dot(np.dot(n1,n2)*(p1-p2),n1))/(1-(np.dot(n1,n2))**2)

        q1=p1-r1*n1
        q2=p2-r2*n2

        jiku=q2-q1

        estR=np.linalg.norm(p1-q1)

        if np.linalg.norm(jiku)<0.01:
            continue

        if estR<minR or maxR<estR:
            continue

        jiku=jiku/np.linalg.norm(jiku)

        #一つ一つの点を見る
        count=0
        for x in pcd.points:
            v=np.array(x-q1)
            d=(np.linalg.norm(v)**2-np.dot(v,jiku)**2)**0.5
            if estR-threshould<=d<=estR+threshould:
                count+=1
        
        if maxinliner<count:
            maxinliner=count
            bestP=q1
            bestJIKU=jiku
            bestR=np.array(estR)
            bestCOUNT=count
       
        itteration+=1
    #print(bestR)
    M=np.block([bestP,bestJIKU])
    M=np.block([M,bestR])
    M=np.block([M,bestCOUNT])
    if i==0:
        result=M
    else:
        result=np.vstack([result,M])
    np.savetxt('stright_REAL_RANSACpy.csv',result,delimiter=',')
    # print("point")
    # print(bestP)
    # print("軸")
    # print(bestJIKU)
   

    #時間計測
elapsed_time = time.time() - start   
print ("演算時間:{0}".format(elapsed_time) + "[sec]")