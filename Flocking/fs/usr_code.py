"""
Ian Kennedy
Lab 3

Simulation with coachbots emulating the brazilian nut effect.



Simulation runs for 3 radii sorting took approximately 1500 simulated seconds till steady state
Simulation runs for 2 radii sorting took approximately 700 simulated seconds till steady state

"""


def usr(robot):
	import struct
	import math
	import timeit

	#Random angle variable
	ran = 0


	#Net x, y and theta values
	net_x = 0
	net_y = 0
	net_t = 0
	
	#robot pose data
	g_x = 0
	g_y = 0
	g_a = 0


	#attraction vector to origin x,y components
	net_x_o = 0
	net_y_o = 0

	#random vectors
	net_x_r = 0
	net_y_r = 0

	# Data of a neighboring robots
	dist = [] #distance from neighbor robot
	d_a = [] # delta angle from neighbor robot
	
	#Received data variables from neighboring robots
	r_x = []
	r_y = []
	r_a = []
	r_id = []

	#Flag to check for identical robot messages in the lists
	same = 0

	# Virtual radii for robots. The lower assigned ids have smaller radii
	if robot.assigned_id==0:
		robot.set_led(100,0,0)
		vr=0.15
	elif robot.assigned_id==1:
		robot.set_led(0,100,0)
		vr = 0.3	
	else:
		robot.set_led(0,0,100)
		vr = 0.4

		
	while True:
		

		#Check time, position and reset flag filtering out identical messages
		time = robot.get_clock()
		pose = robot.get_pose()
		same=0

		
		#The simulation operations in 5 second periods
		time_mod = time%5

		#Send data to other robots
		if pose:
			g_x = pose[0]
			g_y = pose[1]
			g_a = pose[2]
			robot.send_msg(struct.pack('fffi', pose[0], pose[1],pose[2],robot.assigned_id))# send pose x,y in message


		#From 0 to 1 seconds, calculate heading and vector magnitude
		if time_mod<1:
			net_x = 0
			net_y = 0
			net_t = 0

			
			robot.set_vel(0,0)

			#Attraction vector
			theta_o = math.atan2(g_y-0,g_x-0) #heading to origin
			net_x_o = 1*math.cos(theta_o) #attraction to orgin vector components
			net_y_o = 1*math.sin(theta_o)

			#Random vector
			ran = robot.random.randint(-314,314)
			ran /= 100

			net_x_r = 1*math.cos(ran) #random vector components
			net_y_r = 1*math.sin(ran)


			#Check buffer for other messages
			m = robot.recv_msg()

			if len(m) > 0: #Update repulsion data
				m_r = struct.unpack('fffi',m[0][:16])

				for i in range(len(r_x)): # Filter out identical messages
					if abs(m_r[0]-r_x[i])<0.1 and abs(m_r[1]-r_y[i])<0.1:
						same=1
				
				#Append received data to the list
				if same==0 and math.sqrt((g_x - m_r[0])*(g_x - m_r[0]) + (g_y - m_r[1])*(g_y - m_r[1])) < vr:
					r_x.append(m_r[0])
					r_y.append(m_r[1])
					r_a.append(m_r[2])
					r_id.append(m_r[3])			

					#Calculate distances for each neighbor
					dist_temp = math.sqrt((g_x - r_x[len(r_x)-1])**2 + (g_y - r_y[len(r_y)-1])**2)
					dist.append(dist_temp)
					
					#Calculate the respective angles to neighbors
					d_a_temp = math.atan2((m_r[1]-g_y),(m_r[0]-g_x)) - g_a

					if d_a_temp>math.pi:
						d_a_temp = -(2*math.pi-d_a_temp)
					if d_a_temp<-math.pi:
						d_a_temp = 2*math.pi+d_a_temp

					d_a.append(d_a_temp)
		


		#Turn towards the calculated heading
		if 1<time_mod<3:
			
			#Add the three separate vector influences (random motion, attraction to source, repulsion from neighbors)
			net_x-=0.15*net_x_o
			net_y-=0.15*net_y_o	

			net_x-=0.1*net_x_r
			net_y-=0.1*net_y_r		

			for i in range(len(r_x)):
				net_x+=0.3*(g_x-r_x[i])/dist[i]
				net_y+=0.3*(g_y-r_y[i])/dist[i]			

			#Calculate heading		
			try:
				net_t = math.atan2(net_y,net_x)-g_a
			except:
				pass

			if net_t>math.pi:
				net_t -= 2*math.pi
			if net_t<-math.pi:
				net_t += 2*math.pi
			
			try: # Go to heading. Don't set velocity if net_t=0
				robot.set_vel( -int(99*abs(net_t)/net_t) ,+int(99*abs(net_t)/net_t))
			except:
				pass


		#Do forward propulsion based off of neighbor data.
		if 3<time_mod<4:
			if abs(net_x)>0 or abs(net_y)>0:
				robot.set_vel(math.ceil(math.sqrt(net_x*net_x + net_y*net_y)),math.ceil(math.sqrt(net_x*net_x + net_y*net_y)))
			if net_y==0 and net_x==0:
				pass

		#Clear the neighbor data lists
		if time_mod>4:
			robot.set_vel(0,0)
			r_x=[]
			r_y=[]
			r_a=[]
			r_id=[]
			dist=[]
			d_a=[]			






		