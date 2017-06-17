import os
import sys
script_path = os.path.dirname(os.path.realpath(__file__))
os.chdir(os.path.dirname(script_path))
sys.path.append(os.getcwd())


import numpy as np
import math
import time
from objects.pose import Pose
from objects.point import Point

from objects.target import Target
from objects.trajectory import Trajectory
from objects.trajectory_point import TrajectoryPoint
from trajectory.trajectory_planner import TrajectoryPlanner


class TrajectoryTentacle(TrajectoryPlanner):
	def __init__(self):
		self.X = []
		self.Y = []
		self.V_X = []
		self.V_Y = []
		self.A_X = []
		self.A_Y = []
		self.H = []
		self.K = []
		self.time_stamp = []

		self.final_x = 1e7
		self.final_y = 1e7
	

	def generate(self,target,num_points,*args,**kwargs):
		return self.trajectory_planner( target.start.x, target.start.y, target.end.x, target.end.y, target.start.speed, target.end.speed, target.end.heading, target.start.heading,num_points)


	def trajectory_planner(self, initial_wp_x, initial_wp_y, final_wp_x, final_wp_y, initial_wp_v, final_wp_v, final_wp_heading, initial_wp_heading,num_pts):
		#----------old trajectories-------#
		#X_old = []
		#Y_old = []

		#if not (self.final_x == final_wp_x and self.final_y == final_wp_y):
		# X_old = self.X
		# Y_old = self.Y

		#self.final_y = final_wp_y
		#self.final_x = final_wp_x


		#----------nearest point----------#
		# if X_old:
		# 	distance = []
		# 	for i in range(len(X_old)):
		# 		distance.append(math.sqrt((X_old[i] - initial_wp_x)**2 + (Y_old[i] - initial_wp_y)**2))
		# 	desired_idx = distance.index(min(distance))
        #
		# 	initial_wp_x = X_old[desired_idx]
		# 	initial_wp_y = Y_old[desired_idx]


		#------------empty lists----------#
		self.X = []
		self.Y = []
		self.H = []
		self.V_X = []
		self.V_Y = []
		self.A_X = []
		self.A_Y = []
		self.K = []
		self.time_stamp = []




		#-----------------------Build Tentacles----------------------------#
		n = 16  										# Number of sets of speeds
		rho = 1.15
		v_s = 0  										# lowest speed in the set (m/s)
		v_e = 10  										# max speed in the set
		phi = 1.2 * (np.pi / 2)
		q = []
		l = []
		R_j = []  										# Base Radius of curvature
		v_j = []  										# velocity of each tentacle set
		r_k = [[]]  										# radius of curvature of each tentacle
		l_k = [[]] 									 	# length of each tentacle
		theta_k = [[]]



		for j in range(0, 15):
			q.append(float(j) / (n - 1))
			l.append(2 + (7.5 * math.pow(q[j], 0.5)))
			R_j.append(l[j] / (phi * (1 - math.pow(q[j], 0.9))))
			v_j.append(v_s + (v_e - v_s) * (math.pow(q[j], 1.2)))
			r_k.append([])
			l_k.append([])
			theta_k.append([])

			for k in range(0, 81):
				if k < 40:
					r_k[j].append(math.pow(rho, k) * R_j[j])

				elif k > 40:
					r_k[j].append((-math.pow(rho, k - 41)) * R_j[j])

				else:
					r_k[j].append(1e7)


		#--------------Select speed set------------#
		m = 0
		ref = abs(initial_wp_v - v_j[0])
		for j in range(0, 15):
			speed_diff = abs(initial_wp_v - v_j[j])
			if speed_diff < ref:
				ref = speed_diff
				m = j

		if final_wp_y == 0:
			return Trajectory([])
		else:
			#------------cost function-----------------#
			C_alpha = 0.3
			#num_pts = 100
			dt = 0.01
			cost = []
			center_x = []
			center_y = []

			for i in range(81):
				#Get Center
				center_x.append(initial_wp_x + r_k[m][i]*math.sin(initial_wp_heading))
				center_y.append(initial_wp_y - r_k[m][i]*math.cos(initial_wp_heading))

				#get distance
				dist = abs(math.sqrt((center_x[i] - final_wp_x)**2 + (center_y[i] - final_wp_y)**2) - abs(r_k[m][i]))
				#print('idx',i,'d',dist)

				#get angle
				beta = (math.atan((final_wp_y - center_y[i])/(final_wp_x - center_x[i])))
				if beta >= 0:
					if i < 40:
						angle = beta - np.pi/2
					else:
						if beta >= np.pi/2:
							angle = beta - 3*np.pi/2
						else:
							angle = beta + np.pi/2

				else:
					if i > 40:
						angle = beta + np.pi/2
					else:
						if abs(beta) > np.pi/2:
							angle = -3*np.pi/2 - beta
						else:
							angle = beta - np.pi/2

				alpha = abs(angle - final_wp_heading)
				print(final_wp_heading)
				#cost
				cost.append(dist + C_alpha*alpha)

			traj_index = cost.index(min(cost))
			print(traj_index)

			# ax = (final_wp_v*math.cos(final_wp_heading) - initial_wp_v*math.cos(initial_wp_heading))/(num_pts*dt)
			ax = (final_wp_v - initial_wp_v) / (num_pts * dt)
			ay = (final_wp_v*math.sin(final_wp_heading) - initial_wp_v*math.sin(initial_wp_heading))/(num_pts*dt)
			# print(ax, final_wp_v, initial_wp_v, final_wp_heading, initial_wp_heading)

			sx = initial_wp_x*num_pts*dt + 0.5*ax*num_pts*dt*num_pts*dt
			sy = initial_wp_y*num_pts*dt + 0.5*ay*num_pts*dt*num_pts**dt
			s = math.sqrt(sx**2 + sy**2)
			l = math.sqrt((final_wp_x - initial_wp_x)**2 + (final_wp_y - initial_wp_y)**2)
			theta = l/(r_k[m][traj_index])


			#getting points on desired tentacle
			for j in range(num_pts):
				ang = initial_wp_heading + math.copysign(1, r_k[m][traj_index]) * np.pi / 2 - (float(j)/num_pts)*theta
				self.X.append(center_x[traj_index] + abs(r_k[m][traj_index])*math.cos(ang))
				self.Y.append(center_y[traj_index] + abs(r_k[m][traj_index])*math.sin(ang))
				self.A_X.append(ax)
				self.A_Y.append(ay)
				#self.V_X.append(abs(initial_wp_v*math.cos(initial_wp_heading) + ax*(j+20)*dt))
				#self.V_Y.append(abs(initial_wp_v*math.sin(initial_wp_heading) + ay*(j+20)*dt))
				self.V_X.append(final_wp_v)
				self.V_Y.append(0)
				slope = math.atan((final_wp_y - center_y[traj_index])/(final_wp_x - center_x[traj_index]))
				if slope < np.pi/2:
					heading = slope - math.copysign(1,r_k[m][traj_index])*np.pi/2
				else:
					if math.copysign(1,r_k[m][traj_index]) > 0:
						heading = - slope - np.pi/2
					else:
						heading = slope - np.pi/2
				self.H.append(heading)
				self.K.append(1/r_k[m][traj_index])
				self.time_stamp.append(j*dt)
			#print(self.Y[0])
			#print(self.V_X[0])
			# print(self.X)
			return Trajectory([TrajectoryPoint(val[0],val[1],val[2],val[3],val[4],val[5],val[6],-val[7],val[8]) for val in zip(self.X, self.Y, self.V_X, self.V_Y, self.A_X, self.A_Y, self.H, self.K, self.time_stamp)])


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    fig = plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.set_title('Trajectory')

    # create trajectory planner object
    planner = TrajectoryTentacle()
    planner2 = TrajectoryTentacle()

    # test selection of speed set
    # print("v = 0 ->", planner.findSpeedSet(0))
    # print("v = 10 ->", planner.findSpeedSet(10))
    # print("v = 2 ->", planner.findSpeedSet(2))

    # test trajectory generation
    target = Target()
    target.start = Pose(0, 1, 0, 0, 0)
    target.end = Pose(5, 9.5, 0, 0, 0)
    t = planner.generate(target, 50)

    #print ("x",[p.x for p in t.coordinates],"y",[p.y for p in t.coordinates])
    ax1.plot([p.x for p in t.coordinates], [p.y for p in t.coordinates], 'ko')

#    target = Target()
#    target.start = Pose(1, 0.01, 0, 0, 0)
#    target.end = Pose(7, -0.1, 0, 0, 0)
#    t = planner.generate(target, 50)
#    t2 = planner2.generate(target, 50)

    # ax1.plot([p.x for p in t.coordinates], [p.y for p in t.coordinates], 'bo')
    # ax1.plot([p.x for p in t2.coordinates], [p.y for p in t2.coordinates], 'ro')

    # mng = plt.get_current_fig_manager()
    # mng.resize(*mng.window.maxsize())
    plt.show()
