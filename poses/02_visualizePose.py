import cv2 as cv
import pandas as pd
import os
import numpy as np

def read_openpose(path):
	data={}
	f=open(path,'rt')
	lines=f.read().strip().split('\n')
	frame_id=0
	for line in lines:    	
		p=[float(v) for v in line.strip().split()]		
		num_ppl=int(p[0])
		num_skel=int(p[1])
		shape=int(p[2])
		if num_ppl>0:
			arr=np.array(p[3:])
			arr=arr.reshape(num_ppl,num_skel,shape)
			data[frame_id]=arr
		frame_id +=1
	return data

def draw_skeleton_2d(img,skel_data): 
	#skel_joints=[[1,8],[1,2],[1,5],[2,3],[3,4],[5,6],[6,7],[8,9],[9,10],[10,11],[8,12],[12,13],[13,14],[1,0],[0,15],[15,17],[0,16],[16,18],[2,17],[5,18],[14,19],[19,20],[14,21],[11,22],[22,23],[11,24]]
	skel_joints=[[1,8],[1,2],[1,5],[2,3],[3,4],[5,6],[6,7],[8,9],[9,10],[10,11],[8,12],[12,13],[13,14],[1,0],[0,15],[0,16],[14,19],[11,22]]
	
	for v in skel_joints:
		p1=(int(skel_data[0][v[0]][0]),int(skel_data[0][v[0]][1]))
		p2=(int(skel_data[0][v[1]][0]),int(skel_data[0][v[1]][1]))
		cv.line(img,p1,p2,(255,0,0),2)
	for i in range(skel_data[0].shape[0]):
		pt=(int(skel_data[0][i][0]),int(skel_data[0][i][1]))
		cv.circle(img,pt,3,(0,255,0),-1)
		# cv.imshow("preview", img)  
		# key = cv.waitKey()
	return img


if __name__ == '__main__': 
	imgFile="/media/nhquan/60EBA0D0260384D2/hung_mica/datasets/color_l2/FrameC_105.png"
	poseFile="/media/nhquan/60EBA0D0260384D2/hung_mica/poses/color_l2/FrameC_105.txt"
	poseDatas=read_openpose(poseFile)
	print(len(poseDatas))	
	frame = cv.imread(imgFile)		
	img=draw_skeleton_2d(frame,poseDatas[0])				
	cv.imshow("preview", img)  
	key = cv.waitKey()	
	cv.destroyWindow("preview")	