"""
Ian Kennedy
Lab 2
ME409


Source code for the generation of a 2D coordinate system using hop count. This localization and C.S is then applied to construct an N letter out of
256 simulated coachbots. Setting the "smooth" variable to 1 will implement the smoothed hop method, while 0 will implement standard hopcount.

"""

def usr(robot):
    import struct
    import math
    import timeit

    hop_count1 = 255 #255 is uninitialized - left seed hop message (seed1)
    hop_count2 = 255 #uninitialized - right seed hop message (seed2)
    mag_err = 10000.0 #initial error intentionally large 

    

    x_coord=1000 #arbitrarily large x and y coorindates to start
    y_coord=1000


    x_seed1 = 0.0 #Initial seed 1 location
    y_seed1 = 0.0

    x_seed2 = 19.0 #Initial seed 2 location
    y_seed2 = 0.0 
    
    msg_count1=0 #Variables to count messages if smoothed hop method is selected
    msg_count2=0

    hop_smooth1 = 0.0 #Variables to store the smooth hop
    hop_smooth2 = 0.0

    smooth_agg_1 = 0.0 #This pair of variables stores the aggregate sum of hops received by a robot in ord
    smooth_agg_2 = 0.0

    smooth = 0 #set this to 1 for smoothed approach, to 0 for unsmoothed (standard)
    


    while True:
        
        ##################

        if robot.assigned_id == 1: #Seed 1

            robot.set_led(100,100,100)
            d=1
            robot.send_msg(struct.pack('ii',d,1)) #continuously send hop1 message
            
            rcved = robot.recv_msg() #Check for received message
            
            hop_count1 = 0 #Set its own hop value

            if (len(rcved)>0): #Unpack and reset hop value if need be
                msg = struct.unpack('ii',rcved[0][:8])
                rcv_count = msg[0]
                if msg[1]==2:
                    if hop_count2>rcv_count:
                        hop_count2 = (rcv_count+1)
                    
                    if smooth ==1: #Implement smoothing algorithm
                        msg_count1+=1
                        smooth_agg_1+=float(rcv_count)
                        hop_smooth1 = (smooth_agg_1+float(hop_count1))/(float(msg_count1)+1.0)-0.5
                  
                    
                    robot.send_msg(struct.pack('ii',hop_count2,msg[1]))  

        ###################

        if robot.assigned_id == 2: #Seed 2
            
            robot.set_led(100,100,100)
            d=1
            robot.send_msg(struct.pack('ii',d,2)) #Repeatedly send seed 2 hop value

            rcved = robot.recv_msg()

            hop_count2 = 0


            if (len(rcved)>0): #Check for message
                
                msg = struct.unpack('ii',rcved[0][:8])
                rcv_count = msg[0]
                if msg[1]==1: #Adjust hop value if necessary
                    if hop_count1>rcv_count:
                        hop_count1 = (rcv_count+1)

                    if smooth ==1: #Implement smoothing if need be
                        msg_count1+=1
                        smooth_agg_1+=float(rcv_count)
                        hop_smooth1 = (smooth_agg_1+float(hop_count1))/(float(msg_count1)+1.0)-0.5
                  
                    robot.send_msg(struct.pack('ii',hop_count1,msg[1])) 

        ####################


        if robot.assigned_id == 0:
    
            rcved = robot.recv_msg() 

            if (len(rcved) > 0): # Check for received message
               
                msg = struct.unpack('ii',rcved[0][:8])
               
                rcv_count = msg[0]

                if msg[1]==1: #Update hop1

                  if hop_count1>rcv_count:
                    hop_count1 = (rcv_count+1)
                  
                  msg_count1+=1 #Smoothing algorithm 
                  smooth_agg_1+=float(rcv_count)
                  hop_smooth1 = (smooth_agg_1+float(hop_count1))/(float(msg_count1)+1.0)-0.5
                  
                  robot.send_msg(struct.pack('ii',hop_count1,msg[1]))

                if msg[1]==2: #Update hop2
                  if hop_count2>rcv_count:
                    hop_count2 = (rcv_count+1)


                  msg_count2+=1 #Smoothing algorithm
                  smooth_agg_2+=float(rcv_count)
                  hop_smooth2 = (smooth_agg_2+float(hop_count2))/(float(msg_count2)+1.0)-0.5
                    
                  robot.send_msg(struct.pack('ii',hop_count2,msg[1]))

            #These are the gradient based distances to both seeds
            if smooth==0: #If using standard approach
                d_hop1 = float(hop_count1)            
                d_hop2 = float(hop_count2)        

            if smooth==1: #If using smoothed approach
                d_hop1 = float(hop_smooth1)            
                d_hop2 = float(hop_smooth2)  

            i=0.0#starting values to iterate over
            j=-1.0

            if mag_err>.15:
                while i < 22.0:
                    
                    while j < 22.0:
                        d1 = math.sqrt( (i-x_seed1)**2 + (j-y_seed1)**2) #Implementing multilateration algorithm
                        d2 = math.sqrt( (i-x_seed2)**2 + (j-y_seed2)**2)
                        error = abs(d1-d_hop1) + abs(d2-d_hop2)

                        if mag_err>error:
                            x_coord=i
                            y_coord=j
                            mag_err = error
                        j+=0.5
                    
                    j=-1.0
                    i+=0.5
            

            xlow = 5.0 #X value used for left vertical bar in N
            xhigh = 13.0 #X value used for right vertical bar in N

            if smooth ==0 : #Standard condition
                if -2.0<=x_coord<=xlow: #left vertical bar
                    robot.set_led(100,100,100)
            
                elif xlow<=x_coord<=xhigh-7.0 and 5.0<=y_coord<12.0: #Add middle section
                    robot.set_led(100,100,100)
            
                elif  xlow<=x_coord<=xlow+2  and 8.0<=y_coord<=15.0: 
                    robot.set_led(100,100,100)   
                
                elif  xlow+1.5<=x_coord<=xlow+5.0  and 8.0<=y_coord<=12.5: 
                    robot.set_led(100,100,100)   

                elif  xlow+3.5<=x_coord<=xlow+9.0  and 0.0<=y_coord<=10.0: 
                    robot.set_led(100,100,100)   

                elif  xlow+7.5<=x_coord<=xlow+9.0  and 0.0<=y_coord<=7.0: 
                    robot.set_led(100,100,100) 

                elif  xlow+8.5<=x_coord<=xlow+11  and 0<=y_coord<=4.0: 
                    robot.set_led(100,100,100) 
                
                elif xhigh<x_coord: #Add right bar
                    robot.set_led(100,100,100)
    
                else:
                    robot.set_led(0,0,0)

            if smooth ==1: #Smoothed condition
                if -2.0<=x_coord<=xlow: #left vertical bar
                    robot.set_led(100,100,100)
            
                elif xlow<=x_coord<=xhigh-7.0 and 5.0<=y_coord<12.0: #Add middle section
                    robot.set_led(100,100,100)
            
                elif  xlow<=x_coord<=xlow+2  and 8.0<=y_coord<=15.0: 
                    robot.set_led(100,100,100)   
                
                elif  xlow+1.5<=x_coord<=xlow+5.0  and 8.0<=y_coord<=12.5: 
                    robot.set_led(100,100,100)   

                elif  xlow+3.5<=x_coord<=xlow+7.0  and 2.5<=y_coord<=10.0: 
                    robot.set_led(100,100,100)   

                elif  xlow+7.5<=x_coord<=xlow+9.0  and 0.0<=y_coord<=7.0: 
                    robot.set_led(100,100,100) 

                elif  xlow+8.5<=x_coord<=xlow+11  and 0<=y_coord<=4.0: 
                    robot.set_led(100,100,100) 
                
                elif xhigh<x_coord: #Add right bar
                    robot.set_led(100,100,100)
                
                else:
                    robot.set_led(0,0,0)
        