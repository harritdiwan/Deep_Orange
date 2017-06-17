#--------------------------------------------#
# Lane detection with mid-line trajectory generation. Left and right lanes are sorted. 
#
# Harrit Diwan
# 11-18-2016
# ------- Contact Info -------
# Email: hdiwan@clemson.edu
# Phone: 864-553-3973
# Organization: Clemson Univeristy International Center for AUtomotive Research
# Feel free to contact me any time if you have questions.
#--------------------------------------------#

import cv2
import glob
import math
import numpy as np
from PIL import Image 

      
#-------------------------------function definition------------------------------------------------#
#foldername_1 -- folder where your images are stored (for example: '/home/harrit/RCCAR_Conventional/Jia_lab/*.png')
#foldername_2 -- folder where you want to store the images with lane detetction
#Cropvertical_from -- pixel from where you want to start the cropping from in vertical direction
#Cropvertical_till -- pixel till where you want to crop in vertical direction
#Crophorizontal_from -- pixel from where you want to start the cropping from in horizontal direction
#Crophorizontal_till -- pixel till where you want to crop in horizontal direction

def lanedetector(foldername_1,foldername_2,Cropvertical_from, Cropvertical_till, Crophorizontal_from, Crophorizontal_till):
	#--------CONSTANTS---------#
	BLACK    = (   0,   0,   0)
	WHITE    = ( 255, 255, 255) 
	j=1


	#Loop for iterating through dataset
	while 1:


		#------------------------------Lane Detection Module---------------------------------#


		#This code in this form works for postprocessing a dataset. For real time add camera implementation	    
		for img in sorted(glob.glob(foldername_1)): 
			#Reading the images using OpenCV2	
			cv_img = cv2.imread(img)
			#Applying Gaussian filtering 
			cv_img_filtered = cv2.GaussianBlur(cv_img,(5,5),0)
			#Check case for empty images
			if not cv_img_filtered.all():
				continue 
			else:			
				height, width, channels = cv_img_filtered.shape
			#Crop your image as you wish
			cropfrom_y = Cropvertical_from
			croptill_y = Cropvertical_till
			cropheight = croptill_y - cropfrom_y
			cropfrom_x = Crophorizontal_from
			croptill_x = Crophorizontal_till 
			cropwidth = croptill_x - cropfrom_x 
			cv_img_cropped = cv_img_filtered[cropfrom_y:croptill_y, cropfrom_x:croptill_x]
			#RGB to Gray 
			gray_image = cv2.cvtColor(cv_img_cropped, cv2.COLOR_BGR2GRAY)
			#Canny Edge Detection
			image_edges = cv2.Canny(gray_image,50,150,3)
			#Probabilistic Hough Transform
			lines = cv2.HoughLinesP(image_edges, 1, np.pi/180.0, 20, np.array([]), 5, 5)
			#Check case if no line is detected in a frame
			if lines is None:
				continue 
			else:				
				a,b,c = lines.shape
		
			#Real Time matrices for each frame
			X = []
			Y = []
			X_R = []
			Y_R = []
			X_L = []
			Y_L = []
			X_c = []
			Y_c = []
			X_c = []
			Y_c = []
			slope_c = 0	
	
			for x in range(0, len(lines)):
		    		for x1,y1,x2,y2 in lines[x]:
					#Writing detected lanes on the images				
					#cv2.line(cv_img_cropped,(x1,y1),(x2,y2),(0,255,0),1)
					X.append(x1)
					X.append(x2)
					Y.append(y1)
					Y.append(y2)

			#-----------------------------------------------------------------------------------------#




			#--------------------Segmentation into left and right lane module-------------------------#
					#If a point lies on left half of screen classify it as left 
					if x1 < cropwidth/2:
						X_L.append(x1)
						Y_L.append(y1)
						#Check case for sharp turns. Analyzing the top 1/3rd part of screen. If a point lies near half of the screen, add it to a critical matrix
						if y1 < cropheight/3:
							if abs(x1 - cropwidth/2) < 6:
								X_c.append(x1)
								Y_c.append(y1)
								slope_c = math.degrees(math.atan((y2-y1)/(x2-x1)))
					
					if x2 < cropwidth/2:
						X_L.append(x2)
						Y_L.append(y2)
						#Check case for sharp turns. Analyzing the top 1/3rd part of screen. If a point lies near half of the screen, add it to a critical matrix
						if y2 < cropheight/3:
							if abs(x2 - cropwidth/2) < 6:
								X_c.append(x2)
								Y_c.append(y2)
								slope_c = math.degrees(math.atan((y2-y1)/(x2-x1)))

						#cv2.line(cv_img_cropped,(x1,y1),(x2,y2),(255,0,0),1)

					#Right line classification
					if x1 >= cropwidth/2:
						X_R.append(x1)
						Y_R.append(y1)
						#Check case for sharp turns. Analyzing the top 1/3rd part of screen. If a point lies near half of the screen, add it to a critical matrix
						if y1 < cropheight/3:
							if abs(x1 - cropwidth/2) < 6:
								X_c.append(x1)
								Y_c.append(y1)
								slope_c = math.degrees(math.atan((y2-y1)/(x2-x1)))
			
					if x2 >= cropwidth/2:
						X_R.append(x2)
						Y_R.append(y2)
						#Check case for sharp turns. Analyzing the top 1/3rd part of screen. If a point lies near half of the screen, add it to a critical matrix
						if y2 < cropheight/3:
							if abs(x2 - cropwidth/2) < 6:
								X_c.append(x2)
								Y_c.append(y2)
								slope_c = math.degrees(math.atan((y2-y1)/(x2-x1)))					

						#cv2.line(cv_img_cropped,(x1,y1),(x2,y2),(0,0,255),1)

			if not slope_c == 0:
				if not Y_L: 
					slope_ref_L = 45		
				else: 
					slope_ref_L = math.degrees(math.atan((max(Y_L)-min(Y_c))/(X_L[Y_L.index(max(Y_L))]-cropwidth/2)))

				if not Y_R: 
					slope_ref_R = 45		
				else: 
					slope_ref_R = math.degrees(math.atan((max(Y_R)-min(Y_c))/(X_R[Y_R.index(max(Y_R))]-cropwidth/2)))		

			#Tackling the sharp turn critical cases. If lane is truning right, remove falsely setected points in the right set of points and add them to the left set of points		
			if slope_c < 0:
				for xr, yr in zip(X_R,Y_R):
					if yr <= min(Y_c) and X_R[Y_R.index(yr)] > cropwidth/2:
						slope_i = math.degrees(math.atan((yr-min(Y_c))/(X_R[Y_R.index(yr)]-cropwidth/2)))
						if abs(slope_i) > abs(slope_ref_R) or X_R[Y_R.index(yr)] in X_c:
							X_L.append(X_R[Y_R.index(yr)])		
							X_R.remove(X_R[Y_R.index(yr)])	
							Y_L.append(yr)					
							Y_R.remove(yr)
					
					
			#Tackling the sharp turn critical cases. If lane is truning left, remove falsely setected points in the left set of points and add them to the right set of points
			if slope_c > 0:
				for xl, yl in zip(X_L,Y_L):
					if yl <= min(Y_c) and X_L[Y_L.index(yl)] < cropwidth/2:
						slope_i = math.degrees(math.atan((yl-min(Y_c))/(X_L[Y_L.index(yl)]-cropwidth/2)))
						if abs(slope_i) > abs(slope_ref_L) or X_L[Y_L.index(yl)] in X_c:
							X_R.append(X_L[Y_L.index(yl)])	
							X_L.remove(X_L[Y_L.index(yl)])	
							Y_R.append(yl)	
							Y_L.remove(yl)
			
			#Dividing the frame vertically into six parts and averaging out points in each part for left set of points								  	
			n1 = 0
			y_sum1 = 0
			x_sum1 = 0
			n2 = 0
			y_sum2 = 0
			x_sum2 = 0
			n3 = 0
			y_sum3 = 0
			x_sum3 = 0
			n4 = 0
			y_sum4 = 0
			x_sum4 = 0
			n5 = 0
			y_sum5 = 0
			x_sum5 = 0
			n6 = 0
			y_sum6 = 0
			x_sum6 = 0
			for y, x in zip(Y_L,X_L):
				if y < cropheight/6: 
					n1 += 1
					y_sum1 += y
					x_sum1 += x	
				elif y < cropheight/3 and y >= cropheight/6: 
					n2 += 1
					y_sum2 += y
					x_sum2 += x
				elif y < cropheight/2 and y >= cropheight/3: 
					n3 += 1
					y_sum3 += y
					x_sum3 += x
				elif y < 2*cropheight/3 and y >= cropheight/2: 
					n4 += 1
					y_sum4 += y
					x_sum4 += x
				elif y < 5*cropheight/6 and y >= 2*cropheight/3: 
					n5 += 1
					y_sum5 += y
					x_sum5 += x
				elif y <= cropheight and y >= 5*cropheight/6: 
					n6 += 1
					y_sum6 += y
					x_sum6 += x

			if not n1 == 0:				
				y_av1_l = y_sum1/n1
				x_av1_l = x_sum1/n1
		
			if not n2 == 0:	
				y_av2_l = y_sum2/n2
				x_av2_l = x_sum2/n2
			
			if not n3 == 0:					
				y_av3_l = y_sum3/n3
				x_av3_l = x_sum3/n3
			
			if not n4 == 0:					
				y_av4_l = y_sum4/n4
				x_av4_l = x_sum4/n4	

			if not n5 == 0:
				y_av5_l = y_sum5/n5
				x_av5_l = x_sum5/n5

			if not n6 == 0:
				y_av6_l = y_sum6/n6
				x_av6_l = x_sum6/n6


			#Handling cases for left points in which there is no point in a specific part. If no point is detected then an averaged point is generated by interpolation from the adjacent parts.
			if n1 == 0: 
				#The vertical height of averaged point is the vertical middle point of the respective part 
				y_av1_l = cropheight/12
				n1 = 1		
				if not n2 == 0:
					x_av1_l = x_av2_l 
				else:
					if not n3 == 0:
						x_av1_l = x_av3_l
					else: 
						if not n4 == 0:
							x_av1_l = x_av4_l
						else:
							if not n5 == 0:
								x_av1_l = x_av5_l 
							else:
								if not n6 == 0:
									x_av1_l = x_av6_l
								else:
									x_av1_l = width/4
	
			if n2 == 0:
				y_av2_l = cropheight/4
				n2 = 1
				if not n3 == 0:
					x_av2_l = (x_av1_l + x_av3_l)/2
				else: 
					if not n4 == 0:
						x_av2_l = (x_av1_l + x_av4_l)/2
					else:
						if not n5 == 0:
							x_av2_l = (x_av1_l + x_av5_l)/2 
						else:
							if not n6 == 0:
								x_av2_l = (x_av1_l + x_av6_l)/2
							else:
								x_av2_l = x_av1_l   						

			if n3 == 0: 
				y_av3_l = 5*cropheight/12
				n3 = 1
				if not n4 == 0:
					x_av3_l = (x_av2_l + x_av4_l)/2
				else:
					if not n5 == 0:
						x_av3_l = (x_av2_l + x_av5_l)/2 
					else:
						if not n6 == 0:
							x_av3_l = (x_av2_l + x_av6_l)/2
						else:
							x_av3_l = x_av2_l  
	 						
			if n4 == 0:
				y_av4_l = 7*cropheight/12
				n4 = 1
				if not n5 == 0:
					x_av4_l = (x_av3_l + x_av5_l)/2 
				else:
					if not n6 == 0:
						x_av4_l = (x_av3_l + x_av6_l)/2
					else:
						x_av4_l = x_av3_l

			if n5 == 0: 
				y_av5_l = 9*cropheight/12
				n5 = 1
				if not n6 == 0:
					x_av5_l = (x_av4_l + x_av6_l)/2
				else:
					x_av5_l = x_av4_l
	
			if n6 == 0:
				y_av6_l = 11*cropheight/12
				n6 = 1
				x_av6_l = x_av5_l

			#Drawing the averaged lines for left side
			#cv2.line(cv_img_cropped,(x_av1_l,y_av1_l),(x_av2_l,y_av2_l),(255,255,0),3)
			#cv2.line(cv_img_cropped,(x_av2_l,y_av2_l),(x_av3_l,y_av3_l),(255,255,0),3)
			#cv2.line(cv_img_cropped,(x_av3_l,y_av3_l),(x_av4_l,y_av4_l),(255,255,0),3)
			#cv2.line(cv_img_cropped,(x_av4_l,y_av4_l),(x_av5_l,y_av5_l),(255,255,0),3)
			#cv2.line(cv_img_cropped,(x_av5_l,y_av5_l),(x_av6_l,y_av6_l),(255,255,0),3)



			#Dividing the frame vertically into six parts and averaging out points in each part for right set of points
			n1 = 0
			y_sum1 = 0
			x_sum1 = 0
			n2 = 0
			y_sum2 = 0
			x_sum2 = 0
			n3 = 0
			y_sum3 = 0
			x_sum3 = 0
			n4 = 0
			y_sum4 = 0
			x_sum4 = 0
			n5 = 0
			y_sum5 = 0
			x_sum5 = 0
			n6 = 0
			y_sum6 = 0
			x_sum6 = 0
			for y, x in zip(Y_R,X_R):
				if y < cropheight/6: 
					n1 += 1
					y_sum1 += y
					x_sum1 += x	
				elif y < cropheight/3 and y >= cropheight/6: 
					n2 += 1
					y_sum2 += y
					x_sum2 += x
				elif y < cropheight/2 and y >= cropheight/3: 
					n3 += 1
					y_sum3 += y
					x_sum3 += x
				elif y < 2*cropheight/3 and y >= cropheight/2: 
					n4 += 1
					y_sum4 += y
					x_sum4 += x
				elif y < 5*cropheight/6 and y >= 2*cropheight/3: 
					n5 += 1
					y_sum5 += y
					x_sum5 += x
				elif y <= cropheight and y >= 5*cropheight/6: 
					n6 += 1
					y_sum6 += y
					x_sum6 += x

			if not n1 == 0:				
				y_av1_r = y_sum1/n1
				x_av1_r = x_sum1/n1
		
			if not n2 == 0:	
				y_av2_r = y_sum2/n2
				x_av2_r = x_sum2/n2
			
			if not n3 == 0:					
				y_av3_r = y_sum3/n3
				x_av3_r = x_sum3/n3
			
			if not n4 == 0:					
				y_av4_r = y_sum4/n4
				x_av4_r = x_sum4/n4	

			if not n5 == 0:
				y_av5_r = y_sum5/n5
				x_av5_r = x_sum5/n5

			if not n6 == 0:
				y_av6_r = y_sum6/n6
				x_av6_r = x_sum6/n6
	
			#Handling cases for left points in which there is no point in a specific part. If no point is detected then an averaged point is generated by interpolation from the adjacent parts.
			if n1 == 0: 
				y_av1_r = cropheight/12
				n1 = 1		
				if not n2 == 0:
					x_av1_r = x_av2_r 
				else:
					if not n3 == 0:
						x_av1_r = x_av3_r
					else: 
						if not n4 == 0:
							x_av1_r = x_av4_r
						else:
							if not n5 == 0:
								x_av1_r = x_av5_r 
							else:
								if not n6 == 0:
									x_av1_r = x_av6_r
								else:
									x_av1_r = width/4
	
			if n2 == 0:
				y_av2_r = cropheight/4
				n2 = 1
				if not n3 == 0:
					x_av2_r = (x_av1_r + x_av3_r)/2
				else: 
					if not n4 == 0:
						x_av2_r = (x_av1_r + x_av4_r)/2
					else:
						if not n5 == 0:
							x_av2_r = (x_av1_r + x_av5_r)/2 
						else:
							if not n6 == 0:
								x_av2_r = (x_av1_r + x_av6_r)/2
							else:
								x_av2_r = x_av1_r   						

			if n3 == 0: 
				y_av3_r = 5*cropheight/12
				n3 = 1
				if not n4 == 0:
					x_av3_r = (x_av2_r + x_av4_r)/2
				else:
					if not n5 == 0:
						x_av3_r = (x_av2_r + x_av5_r)/2 
					else:
						if not n6 == 0:
							x_av3_r = (x_av2_r + x_av6_r)/2
						else:
							x_av3_r = x_av2_r  
	 						
			if n4 == 0:
				y_av4_r = 7*cropheight/12
				n4 = 1
				if not n5 == 0:
					x_av4_r = (x_av3_r + x_av5_r)/2 
				else:
					if not n6 == 0:
						x_av4_r = (x_av3_r + x_av6_r)/2
					else:
						x_av4_r = x_av3_r

			if n5 == 0: 
				y_av5_r = 9*cropheight/12
				n5 = 1
				if not n6 == 0:
					x_av5_r = (x_av4_r + x_av6_r)/2
				else:
					x_av5_r = x_av4_r
	
			if n6 == 0:
				y_av6_r = 11*cropheight/12
				n6 = 1
				x_av6_r = x_av5_r

			#Drawing the averaged lines for right side
			#cv2.line(cv_img_cropped,(x_av1_r,y_av1_r),(x_av2_r,y_av2_r),(0,255,255),3)
			#cv2.line(cv_img_cropped,(x_av2_r,y_av2_r),(x_av3_r,y_av3_r),(0,255,255),3)
			#cv2.line(cv_img_cropped,(x_av3_r,y_av3_r),(x_av4_r,y_av4_r),(0,255,255),3)
			#cv2.line(cv_img_cropped,(x_av4_r,y_av4_r),(x_av5_r,y_av5_r),(0,255,255),3)
			#cv2.line(cv_img_cropped,(x_av5_r,y_av5_r),(x_av6_r,y_av6_r),(0,255,255),3)




			#-------------Middle line trajectory generation module---------------#
			xm1 = (x_av1_r + x_av1_l)/2
			ym1 = (y_av1_r + y_av1_l)/2
			xm2 = (x_av2_r + x_av2_l)/2
			ym2 = (y_av2_r + y_av2_l)/2
			xm3 = (x_av3_r + x_av3_l)/2
			ym3 = (y_av3_r + y_av3_l)/2
			xm4 = (x_av4_r + x_av4_l)/2
			ym4 = (y_av4_r + y_av4_l)/2
			xm5 = (x_av5_r + x_av5_l)/2
			ym5 = (y_av5_r + y_av5_l)/2
			xm6 = (x_av6_r + x_av6_l)/2
			ym6 = (y_av6_r + y_av6_l)/2		
		
			#Drawing the middle line trajectory on the image
			#cv2.line(cv_img_cropped,(xm1,ym1),(xm2,ym2),(255,255,255),3)
			#cv2.line(cv_img_cropped,(xm3,ym3),(xm2,ym2),(255,255,255),3)
			#cv2.line(cv_img_cropped,(xm3,ym3),(xm4,ym4),(255,255,255),3)
			#cv2.line(cv_img_cropped,(xm4,ym4),(xm5,ym5),(255,255,255),3)
			#cv2.line(cv_img_cropped,(xm5,ym5),(xm6,ym6),(255,255,255),3)
		

			#--------------------Desired output left and right lanes------------------------------#
			left = [[x_av1_l,y_av1_l],[x_av2_l,y_av2_l],[x_av3_l,y_av3_l],[x_av4_l,y_av4_l],[x_av5_l,y_av5_l],[x_av6_l,y_av6_l]]
			right =[[x_av1_r,y_av1_r],[x_av2_r,y_av2_r],[x_av3_r,y_av3_r],[x_av4_r,y_av4_r],[x_av5_r,y_av5_r],[x_av6_r,y_av6_r]]		
		

			#Writing the processed images to a folder 	
			#cv2.imwrite(foldername_2 + str(j) + '.png',cv_img_cropped)   	#Drawing the detected lines over the original images	
		
			# returning left and right lanes arrays
			return left, right
			j+=1
