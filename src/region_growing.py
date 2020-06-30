import csv
import numpy as np 
import open3d as o3d
import time

start=time.time()
pcd=o3d.io.read_point_cloud("onlypipe_transed_mini.ply")
point=np.asarray(pcd.points)



with open('point_Nvec_lamda3_0.4m.csv') as f:
    reader=csv.reader(f)
    l=[row for row in reader]

lamda3M=np.zeros((len(point),1))

#ここから移植させる
for i in range(len(point)):
    lamda3M[i,0]=np.asarray(l[i][7])

M=np.block([point,lamda3M])

#直線の場所
straightP=np.where(lamda3M==0)[0]
straightM=point[straightP]
sflagM=np.zeros((len(straightP),1))


#エルボの場所
elbowP=np.where(lamda3M==1)[0]
elbowM=point[elbowP]
eflagM=np.zeros((len(elbowP),1))



#直線regiongrowing始まるよ
spcd=o3d.PointCloud()
spcd.points = o3d.Vector3dVector(straightM[:,:3])
spcd_tree = o3d.geometry.KDTreeFlann(spcd)

flag=1
flaged=[]

while True:
    #まだ残ってる点リスト
    zeroP=np.where(sflagM==0)[0]
    if len(zeroP)==0:
        break

    flag=flag+1

    round=[0]
    for i in round:
        #print(i)
        if i==0:
            #新しいところなのでseedを新しく設定
            seed_num=[zeroP[0]]
        else:
            #一つ前のroundの新規点をseed
            seed_num=newaround           

        nowaround=[]
        #round i でのseedpoint周りの点を探す
        for seedp in seed_num:
            [k, idx, dist] = spcd_tree.search_radius_vector_3d(spcd.points[seedp], 0.5)
            for indexN in idx:
                sflagM[indexN,0]=flag
                nowaround.append(indexN)
                
        nowaround=list(set(nowaround))

        #共通項を探す       
        kyoutuu=list(set(nowaround) & set(flaged) )

        #flag済とnewaroundを合体
        flaged.extend(nowaround)  
        flaged=list(set(flaged))

        newaround=nowaround
        #今回のやつからすでにあったものを引く、残ったものが本当に新しいところ
        for kyo in kyoutuu:
            newaround.remove(kyo)

        if len(newaround)==0:
            amount_num=np.count_nonzero(sflagM==flag)
            if amount_num < 600:
                sflagM=np.where(sflagM==flag,1,sflagM)

            break
       
        round.append(i+1)
                
    
straightM=np.block([straightM,sflagM])
np.savetxt('straightP.csv',straightM,delimiter=',')

#エルボregiongrowing始まるよ
spcd=o3d.PointCloud()
spcd.points = o3d.Vector3dVector(elbowM[:,:3])
spcd_tree = o3d.geometry.KDTreeFlann(spcd)


flaged=[]

while True:
    #まだ残ってる点リスト
    zeroP=np.where(eflagM==0)[0]
    if len(zeroP)==0:
        break

    flag=flag+1

    round=[0]
    for i in round:
        #print(i)
        if i==0:
            #新しいところなのでseedを新しく設定
            seed_num=[zeroP[0]]
        else:
            #一つ前のroundの新規点をseed
            seed_num=newaround           

        nowaround=[]
        #round i でのseedpoint周りの点を探す
        for seedp in seed_num:
            [k, idx, dist] = spcd_tree.search_radius_vector_3d(spcd.points[seedp], 0.5)
            for indexN in idx:
                eflagM[indexN,0]=flag
                nowaround.append(indexN)
                
        nowaround=list(set(nowaround))

        #共通項を探す       
        kyoutuu=list(set(nowaround) & set(flaged) )

        #flag済とnewaroundを合体
        flaged.extend(nowaround)  
        flaged=list(set(flaged))

        newaround=nowaround
        #今回のやつからすでにあったものを引く、残ったものが本当に新しいところ
        for kyo in kyoutuu:
            newaround.remove(kyo)

        if len(newaround)==0:
            amount_num=np.count_nonzero(eflagM==flag)
            if amount_num < 600:
                eflagM=np.where(eflagM==flag,1,eflagM)


            break
       
        round.append(i+1)
                
    
elbowM=np.block([elbowM,eflagM])
np.savetxt('elbowP.csv',elbowM,delimiter=',')

#時間計測
elapsed_time = time.time() - start   
print ("演算時間:{0}".format(elapsed_time) + "[sec]")