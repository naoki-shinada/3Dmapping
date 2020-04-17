# -*- coding: utf-8 -*-

import numpy as np
import cv2
import os
from glob import glob

#RGB画像ゆがみ補正
camera_mat =np.matrix([[4097.446703,0,2169.541862],[0,4084.187389,1611.357725],[0,0,1]])
dist_coef = np.matrix([-0.398939962,0.058796819,-0.002891692,-0.004524933,0.181850735])

#RGB画像トリミングパラメータ
x=377
y=105
a=1360.31-217.5*x/y  #上大きく
b=a+512*x/y
c=1903.54-301*x/y    #右小さく
d=c+640*x/y
size=(640,512)

#さび閾値
Rlower=np.array([155,5,30])
Rupper=np.array([195,255,255])

#配管閾値
Llower=np.array([0,0,200])
Lupper=np.array([179,255,255])




#RGB画像の取得

for i in range(5):
    #RGBが偶数
    for fn in glob("*"+str(i*2)+".jpg"):
    #RGBが奇数
    #for fn in glob("*383.jpg"):
    #for fn in glob("*"+str(i*2+1)+".jpg"):
            im = cv2.imread(fn)

            # RGB画像の補正
            undistort_image = cv2.undistort(im, camera_mat, dist_coef)
            dst = undistort_image[int(a):int(b),int(c):int(d)] 
            resizeim=cv2.resize(dst,size)
            cv2.imwrite("../RGB/" + fn ,resizeim)  
            #os.remove("../thermo/"+fn)



            # さびマスク画像
            hsv=cv2.cvtColor(resizeim, cv2.COLOR_BGR2HSV)
            sabi_mask=cv2.inRange(hsv, Rlower, Rupper)
            sabi_mask=255-sabi_mask
            cv2.imwrite("../RUSTmask/sabimask"+fn ,sabi_mask)


            # 配管マスク画像
            light_mask=255-cv2.inRange(hsv, Llower, Lupper)
            mask=cv2.bitwise_and(sabi_mask,light_mask)
            cv2.imwrite("../PIPEmask/pipemask"+fn,mask)
            cv2.imwrite("../LIGHTmask/lightmask"+fn,light_mask)


            #直線探し
            blur1=cv2.bilateralFilter(resizeim,9,35,35)
            blur2=cv2.bilateralFilter(blur1,9,35,35)
            blur3=cv2.bilateralFilter(blur2,9,35,35)
            blur4=cv2.bilateralFilter(blur3,9,35,35)
            blur5=cv2.bilateralFilter(blur4,9,35,35)
            blur6=cv2.bilateralFilter(blur5,9,35,35)
            blur=cv2.bilateralFilter(blur6,9,35,35)
            edges = cv2.Canny(blur,200,500)
            lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/360, threshold=30, minLineLength=150, maxLineGap=12)
            
            #arr=np.array([0])
            if lines is None:
                red_lines_img=resizeim
                arr=np.array([0])
                np.savetxt("../wideM/"+fn.replace(".jpg","lineM.csv"),arr.T,delimiter=",")
                    
            else:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    

            #直線を求める
                    #横線があったら
                    if y1!=y2 and abs((x2-x1)/(y2-y1))>5:
                        red_lines_img = cv2.line(resizeim, (x1,y1), (x2,y2), (0,0,255), 2)
                        arr=np.array([1])
                        #print("横線")
                        np.savetxt("../wideM/"+fn.replace(".jpg","lineM.csv"),arr.T,delimiter=",")
                        break
                    
                    elif y1==y2:
                        red_lines_img = cv2.line(resizeim, (x1,y1), (x2,y2), (0,0,255), 2)
                        arr=np.array([1])
                        #print("横線")
                        np.savetxt("../wideM/"+fn.replace(".jpg","lineM.csv"),arr.T,delimiter=",")
                        break

                    else:
                        red_lines_img= cv2.line(resizeim, (x1,y1), (x2,y2), (0,255,0), 2)
                        arr=np.array([0])
                        #print("縦線")
                        np.savetxt("../wideM/"+fn.replace(".jpg","lineM.csv"),arr.T,delimiter=",")

            cv2.imwrite("../LINE/line"+fn, red_lines_img)
            #np.savetxt("../wideM/"+fn.replace(".jpg","lineM.csv"),arr.T,delimiter=",")
            #cv2.imwrite("../edge"+fn, edges)
            #cv2.imwrite("../line"+fn, red_lines_img)
            #np.savetxt("../"+fn.replace(".jpg","lineM.csv"),arr.T,delimiter=",")
