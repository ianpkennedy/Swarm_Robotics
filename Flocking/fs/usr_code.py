"""
Lab 4 - Robot Flocking 

Ian Kennedy
ME409

"""

def usr(robot):
    import struct
    import math
    import timeit


	#Average position variables for cohesion effect
    coh_x = 0.0
    coh_y = 0.0
    coh_o = 0.0
    
    coh_vec_x = 0.0
    coh_vec_y = 0.0

    v_r_coh = 0.5 #Virtual radius for cohesion sensing

    r_a_coh = []
    r_x_coh = []
    r_y_coh = []
    r_id_coh = []
    same_coh = 0

    #Average heading for alignment variables
    avg_t = 0
    avg_t_del = 0

    #Lists for heading alignment
    r_a_a = []
    r_x_a = []
    r_y_a = []
    r_id_a = []

    gain = 0
    
    #This is the alignment virtual radius. All robots should align to a global average heading
    vr_a = 0.4

    #Flag to check for identical robot messages in the lists for the repulsion and alignment affect
    same = 0

    #robot pose data
    g_x = 0
    g_y = 0
    g_a = 0


	#attraction vector to origin x,y components
    mig_x = 0
    mig_y = 0
    mig_o = 0
    dist_o = 0

	#This is the virtual radius used for the robot avoidance maneuver
    vr=0.3

	# Data of neighboring robots used for  

    dist = [] #distance from neighbor robot
    d_a = [] # delta angle from neighbor robot
	
    r_x = []
    r_y = []
    r_a = []
    r_id = []

    net_x = 0
    net_y = 0
    net_t = 0

    while True:
        
        #Acquire time, pose and phase variable for state behavior of robot
        t = robot.get_clock()
        pose = robot.get_pose()
        phase = t % 5
        
        #Check for a pose, get x,y and theta data
        if pose:
            g_x = pose[0]
            g_y = pose[1]
            g_a = pose[2]
            dist_o = math.sqrt((g_x-0.0)**2 + (g_y-0.0)**2) 
            robot.send_msg(struct.pack('fffi', pose[0], pose[1],pose[2],robot.id)) #Send pose x,y in message

        # Check for messages from neighbors
        m = robot.recv_msg() 


        # Collect migration force, cohesion force and repulsion force data in this state
        if phase<1:
            robot.set_vel(0,0)
            robot.set_led(0,0,100)

            mig_o = math.atan2(g_y-0,g_x-0) #heading to origin
            mig_x = 1*math.cos(mig_o) #attraction to orgin vector components
            mig_y = 1*math.sin(mig_o)
            
            # When received messages exist, add
            if len(m)>0:

                # unpack received data
                m_r = struct.unpack('fffi',m[0][:16])

                #Filter out repeated data
                for i in range(len(r_x)):
                    if m_r[0]==r_x[i] and m_r[1]==r_y[i]:
                        same=1
                
                for i in range(len(r_x_coh)):
                    if m_r[0]==r_x_coh[i] and m_r[1]==r_y_coh[i]:
                        same_coh=1

                #Exclude data outside of virtual communication range     
                if same_coh==0 and math.sqrt((g_x - m_r[0])**2 + (g_y - m_r[1])**2) < v_r_coh:
                    r_x_coh.append(m_r[0])
                    r_y_coh.append(m_r[1])
                    r_a_coh.append(m_r[2])
                    r_id_coh.append(m_r[3])

                if same==0 and math.sqrt((g_x - m_r[0])**2 + (g_y - m_r[1])**2) < vr:
                    r_x.append(m_r[0])
                    r_y.append(m_r[1])
                    r_a.append(m_r[2])
                    r_id.append(m_r[3])


                    #For repulsion, add calculate cartesian X,Y distance to neighbors
                    dist_temp = math.sqrt((g_x - r_x[len(r_x)-1])**2 + (g_y - r_y[len(r_y)-1])**2)
                    dist.append(dist_temp)

                    #Calculate heading angle with neighbors 
                    d_a_temp = math.atan2((m_r[1]-g_y),(m_r[0]-g_x)) - g_a
                    if d_a_temp>math.pi:
                        d_a_temp = -(2*math.pi-d_a_temp)
                    if d_a_temp<-math.pi:
                        d_a_temp = 2*math.pi+d_a_temp
                    d_a.append(d_a_temp)

            #Reset flags detecting repetitive messages
            same = 0
            same_coh = 0


            # The calculate average location of neighbors
            if len(r_x_coh)>0:
                coh_x = sum(r_x_coh)/len(r_x_coh)
                coh_y = sum(r_y_coh)/len(r_y_coh)
                coh_o = math.atan2(g_y-coh_y,g_x-coh_x) 
                coh_vec_x = 1*math.cos(coh_o)
                coh_vec_y = 1*math.sin(coh_o) #Calculate the cohesion force X,Y components


#after moving forward you could calculate a new heading agv t delta and do a forward move based off of that before start of new cycle
#try making the alignment virtual radius smaller





        if 1<phase<2:
            robot.set_led(100,100,0)

            net_x=-0.03*dist_o*mig_x 
            net_y=-0.03*dist_o*mig_y	

            net_x-=0.02*coh_vec_x
            net_y-=0.02*coh_vec_y

            for i in range(len(r_x)):
                net_x+=0.01*(g_x-r_x[i])/dist[i]
                net_y+=0.01*(g_y-r_y[i])/dist[i]

            #Calculate heading
            try:
                net_t = math.atan2(net_y,net_x) - g_a
            except:
                pass
            
            if net_t>math.pi:
                net_t -= 2*math.pi
            if net_t<-math.pi:
                net_t += 2*math.pi    


            #Go to heading
            turn = int(99*abs(net_t)/net_t)
            try: # Go to heading. Don't set velocity if net_t=0. scale by dist_0
                robot.set_vel( -turn ,+turn)
            except:
                pass

        # In this phase, the robots move towards their target 
        if 2.0<phase<3.0:
            robot.set_led(0,100,100)
            robot.set_vel(int(1000*dist_o*math.sqrt(net_x*net_x + net_y*net_y)),int(1000*dist_o*math.sqrt(net_x*net_x + net_y*net_y)) )


        # Calculate average heading alignment for robots
        if 3.0<phase<4.0:
            robot.set_led(0,100,0)
            robot.set_vel(0,0)
            

            #Check for new messages
            if len(m)>0:
                m_r = struct.unpack('fffi',m[0][:16])
                
                #Filter out repetitive messages
                for i in range(len(r_x_a)):
                    if m_r[0]==r_x_a[i] and m_r[1]==r_y_a[i]:
                        same=1

                #Append new data
                if same==0 and math.sqrt((g_x - m_r[0])**2 + (g_y - m_r[1])**2) < vr_a:
                    r_x_a.append(m_r[0])
                    r_y_a.append(m_r[1])                    
                    r_a_a.append(m_r[2])
                    r_id.append(m_r[3])

            same  = 0
            
            #Calculate average heading
            if len(r_a_a)>0:
                try:
                    avg_t = sum(r_a_a)/len(r_a_a)
                except:
                    avg_t = 0

            #Calculate average delta angle from average heading
            avg_t_del =  avg_t - g_a    
                                        
        #Implement heading alignment between robots
        if 4.0<phase<4.9:
            robot.set_led(100,100,100)
            gain = abs(int(150*avg_t_del))


            #Check which direction to align to
            if avg_t_del>0.1:
                robot.set_vel(5,5+gain)

            elif avg_t_del<-0.1:
                robot.set_vel(5+gain,5)
            else:
                robot.set_vel(5,5)

        if phase>4.9:
            #Clear all the data
            net_x = 0
            net_y = 0
            net_t=0
            r_x=[]
            r_y=[]
            r_a=[]
            r_id=[]
            dist=[]
            d_a=[]
            r_a_a=[]
            r_x_a=[]
            r_y_a=[]
            r_x_coh=[]
            r_y_coh=[]
            r_a_coh=[]
            r_id_coh=[]

          
