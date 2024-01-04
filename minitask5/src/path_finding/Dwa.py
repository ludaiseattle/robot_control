import numpy as np
import matplotlib.pyplot as plt
import math

"""
just use it as an exists model

"""
class Info():
	def __init__(self):
		self.v_min = -0.2   # min speed
		self.v_max = 0.2     # max speed
		self.w_max = 20.0 * math.pi / 180.0   #min angular speed
		self.w_min = -20.0 * math.pi / 180.0  #max angular speed
		self.vacc_max = 0.3         
		self.wacc_max = 20.0 * math.pi / 180.0
		self.v_reso = 0.1
		self.w_reso = 0.5 * math.pi / 180.0
		self.radius = 0.3
		self.dt = 0.1
		self.predict_time = 4.0
		self.goal_factor = 1.0
		self.vel_factor = 1.0
		self.traj_factor = 1.0

# motion model, location(x,y), theta, linear speed, angular speed(w)
def motion_model(x,u,dt):
	# robot motion model: x,y,theta,v,w
	x[0] += u[0] * dt * math.cos(x[2])
	x[1] += u[0] * dt * math.sin(x[2])
	x[2] += u[1] * dt
	x[3] = u[0]
	x[4] = u[1]
	return x

# produce speed space
def vw_generate(x,info):
	#generate v,w window for traj prediction
	Vinfo = [info.v_min,info.v_max,
	         info.w_min,info.w_max]

	Vmove = [x[3] - info.vacc_max * info.dt,
	         x[3] + info.vacc_max * info.dt,
			 x[4] - info.wacc_max * info.dt,
			 x[4] + info.wacc_max * info.dt]

	vw = [max(Vinfo[0],Vmove[0]),min(Vinfo[1],Vmove[1]),
	      max(Vinfo[2],Vmove[2]),min(Vinfo[3],Vmove[3])]
	
	return vw

#forecast trajection by current location and speed
def traj_cauculate(x,u,info):
	ctraj = np.array(x)
	xnew = np.array(x) 
	time = 0

	while time <= info.predict_time: 
		xnew = motion_model(xnew,u,info.dt)
		ctraj = np.vstack((ctraj,xnew))
		time += info.dt

	return ctraj

def dwa_core(x,u,goal,info,obstacles):
	# the kernel of dwa
	vw = vw_generate(x,info)
	best_ctraj = np.array(x)
	min_score = 10000.0

        #v,w are limited in speed space
        print("33333")
	for v in np.arange(vw[0],vw[1],info.v_reso):
		for w in np.arange(vw[2],vw[3],info.w_reso):
			ctraj = traj_cauculate(x,[v,w],info)
                        #evaluation
			goal_score = info.goal_factor * goal_evaluate(ctraj,goal)
			vel_score = info.vel_factor * velocity_evaluate(ctraj,info)
			traj_score = info.traj_factor * traj_evaluate(ctraj,obstacles,info)
			ctraj_score = goal_score + vel_score + traj_score

                        # the less score the best trajection
                        #score: the distance to object+speed+object
			if min_score >= ctraj_score:
				min_score = ctraj_score 
				u = np.array([v,w])
				best_ctraj = ctraj
        print("4444")
	return u,best_ctraj

def goal_evaluate(traj,goal):
	#cauculate current pose to goal with euclidean distance
	goal_score = math.sqrt((traj[-1,0]-goal[0])**2 + (traj[-1,1]-goal[1])**2)
	return goal_score

def velocity_evaluate(traj,info):
	#cauculate current velocty score
	vel_score = info.v_max - traj[-1,3]
	return vel_score

def traj_evaluate(traj,obstacles,info):
	#evaluate current traj with the min distance to obstacles
	min_dis = float("Inf")
	for i in range(len(traj)):
		for ii in range(len(obstacles)):
			current_dist = math.sqrt((traj[i,0] - obstacles[ii,0])**2 + (traj[i,1] - obstacles[ii,1])**2)

			if current_dist <= info.radius:
				return float("Inf")

			if min_dis >= current_dist:
				min_dis = current_dist
	
	return 1 / min_dis

