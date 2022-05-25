import time
import mraa
import threading
import canopen
import sys
sys.path.append("../odometry")
sys.path.append("../commands")
import odometry as odom
import command as comm

# Global Variables

# Right wheel radius
Rright = odom.rightWheelDiam/2 # [mm]
# Left Wheel Radius
Rleft = odom.leftWheelDiam/2 # [mm]
# Distance between the two rear wheels
L = odom.wheelAxis # [mm]
b = L/2 # [mm]
# Distance between the two front wheels
Rf = odom.frontAxis # [mm]
# Distance between the rear wheels axle and the front wheels axle
Rl = odom.axisDist # [mm]
# Reduction factor
G = odom.reductionRatio
# Maximum motor speed
MaxMotorSpeed = odom.maxMotorSpeed # [rpm]
# dead man pins
deadPin1 = mraa.Gpio(13)
deadPin1.dir(mraa.DIR_OUT)
deadPin2 = mraa.Gpio(15)
deadPin2.dir(mraa.DIR_OUT)

#######################################
def norm_angle(angle): # angle em graus
   angle = angle % 360
   if angle > 180:
      angle = angle - 360
   elif angle < -180:
      angle = angle + 360
   return angle

#######################################
def stopWheelchair(node):
   count = 0;
   # set speed to zero
   node.sdo[0x2002][1].raw = 0 
   node.sdo[0x2002][2].raw = 0 
   odom.lock.acquire()
   while True:
      if (abs(odometry.RightRotation) < 3.0 and abs(odometry.LeftRotation) < 3.0) or count == 8:
         dead1Pin.write(1)
         dead2Pin.write(1)
         break
      time.sleep(0.5)
      count += 1
   odom.lock.release()

######################################
def calculate_velocities(v,w,alpha,P,Pinitial):
   epsilon = 0.0001
   if abs(w) < epsilon:
      # Controller gains
      K = [0.0001,0.1,0.1,0]
      if v < 0:
         K[0] = -K[0]
         K[2] = -K[2]
      
      # Calculate the coefficients of the line equation
      A = math.tan((math.pi/180)*Pinitial[2])
      B = -1
      C = -math.tan((math.pi/180)*Pinitial[2])*Pinitial[0]+Pinitial[1]
      
      # Calculate the steady state angles for the caster wheels
      if v >= 0:
         alpha_ss = [0,0] # [[º],[º]]
      else:
         alpha_ss = [180,180] # [[º],[º]]
      
      # Calculate the distance till the line
      D = abs(A*P[0]+B*P[1]+C)/math.sqrt(A**2+B**2)
      if P[1] < Pinitial[1]:
         D = -D
      
      # Calculate the difference between the desired heading and the current heading
      th = P[2]-Pinitial[2] # [º]
   else:
      # Controller gains
      K = [0.0001,0.0001,0.1,0]
      
      # Calculate the curvature radius of the curve
      r = b*(theta_r_dot*Rright+theta_l_dot*Rleft)/(theta_r_dot*Rright-theta_l_dot*Rleft)
      
      # Calculate the position of the Instanteneous Center of Rotation (ICR)
      ICR = [Pinitial[0] - r*math.sin((math.pi/180)*Pinitial[2]), Pinitial[1] + r*math.cos((math.pi/180)*Pinitial[2])]
      
      # Calculate the steady state angles for the caster wheels
      alpha_ss = [(180/math.pi)*(math.atan2(Rl,r+Rf/2)), (180/math.pi)*(math.atan2(Rl,r-Rf/2))] # [[º],[º]]
      
      # Calculate the distance till the curve
      D = r-math.sqrt((P[0]-ICR[0])**2 + (P[1]-ICR[1])**2)
      
      # Calculate the difference between the desired heading and the current heading
      phi = (180/math.pi)*math.atan2(P[1]-ICR[1],P[0]-ICR[0]) # [º]
      th = -(90+phi-P[2])
   
   # Calculate initial Delta
   if (((alpha[0]-alpha_ss[0])%180) > 90 or ((alpha[1]-alpha_ss[1])%180) > 90):
      K[0] = 0
   else:
      K[0] = 0.0001
   Delta = K[0]*((alpha[0]-alpha_ss[0])+(alpha[1]-alpha_ss[1]))+K[1]*th+K[2]*D
   
   if abs(w) < epsilon:
      theta_r_dot = (v-Delta)/R
      theta_l_dot = (v+Delta)/R
   else:
      if w >= 0:
         theta_r_dot = (v+((w*L)/2)-Delta)/R
         theta_l_dot = (v-((w*L)/2)+Delta)/R
      else:
         theta_r_dot = (v+((w*L)/2)+Delta)/R
         theta_l_dot = (v-((w*L)/2)-Delta)/R
   
   return [theta_r_dot,theta_l_dot]

######################################

def heading(delta_th,node):
   # Grab data
   # Wheelchair initial position and caster wheels initial orientation
   odom.lock.acquire()
   px = odom.poseX
   py = odom.poseY
   pth = odom.poseTh # [rad]
   alpha_r = odom.RightCaster
   alpha_l = odom.LeftCaster
   odom.lock.release()
   Pinitial = [px,py,(180/math.pi)*pth] # Pose inicial [x,y,th] -> [[mm],[mm],[º]]
   alpha_initial = [alpha_r,alpha_l] # [[º],[º]]
   
   # Linear velocity for the wheelchair
   v = 0
   
   # Sets the angular velocity for the wheelchair
   w = 0.1
   if delta_th < 0:
      w = -w
   
   # Initial position
   P = Pinitial
   
   # Minimum acceptable error for the angle
   #epsilon = 0.0175 # [rad] -> 1º approximately
   epsilon = 1 # [º]
   
   # Control loop
   while abs(delta_th-(P[2]-Pinitial[2])) > epsilon:
      # Verify if a new command was issued
      if comm.event.is_set():
         return
      
      # Grab wheelchair position and caster wheels orientation
      odom.lock.acquire()
      px = odom.poseX
      py = odom.poseY
      pth = odom.poseTh # [rad]
      alpha_r = odom.RightCaster
      alpha_l = odom.LeftCaster
      odom.lock.release()
      P = [px,py,(180/math.pi)*pth] # Pose inicial [x,y,th] -> [[mm],[mm],[º]]
      alpha = [alpha_r,alpha_l] # [[º],[º]]
      
      # Calculate the velocities and send them to the rear wheels controller
      [wr,wl] = calculate_velocities(v,w,alpha,P,Pinitial)
      
      # SEND HERE THE VELOCITIES TO THE CONTROLLER!!
      rightMotorSpeed = (60/(2*math.pi))*G*wr # [rpm]
      leftMotorSpeed = (60/(2*math.pi))*G*wl # [rpm]
      node.sdo[0x2002][2].raw = -round(rightMotorSpeed) # right motor
      node.sdo[0x2002][1].raw = round(leftMotorSpeed) # left motor

   # STOP THE WHEELCHAIR!!!!
   stopWheelchair(node)

######################################

def displ(delta_s,node):
   # Grab data
   # Wheelchair initial position and caster wheels initial orientation
   odom.lock.acquire()
   px = odom.poseX
   py = odom.poseY
   pth = odom.poseTh # [rad]
   alpha_r = odom.RightCaster
   alpha_l = odom.LeftCaster
   odom.lock.release()
   Pinitial = [px,py,(180/math.pi)*pth] # Pose inicial [x,y,th] -> [[mm],[mm],[º]]
   alpha_initial = [alpha_r,alpha_l] # [[º],[º]]
   
   # Sets the linear velocity for the wheelchair
   v = 0.1
   if delta_s < 0:
      v = -v
      delta_s = -delta_s
   
   # Angular velocity for the wheelchair
   w = 0
   
   # Controller gains
   K = [0.0001,0.1,0.1,0] # Controls the path
   if v < 0:
      K[0] = -K[0]
      K[2] = -K[2]
   
   # Initial position
   P = Pinitial
   
   # Minimum acceptable error for the distance
   #epsilon = 0.01 # [m]
   epsilon = 10 # [mm]
   
   # Control loop
   while abs(delta_s-math.sqrt((P[0]-Pinitial[0])**2+(P[1]-Pinitial[1])**2)) > epsilon:
      # Verify if a new command was issued
      if comm.event.is_set():
         return
      
      # Grab wheelchair position and caster wheels orientation
      odom.lock.acquire()
      px = odom.poseX
      py = odom.poseY
      pth = odom.poseTh # [rad]
      alpha_r = odom.RightCaster
      alpha_l = odom.LeftCaster
      odom.lock.release()
      P = [px,py,(180/math.pi)*pth] # Pose inicial [x,y,th] -> [[mm],[mm],[º]]
      alpha = [alpha_r,alpha_l] # [[º],[º]]
      
      # Calculate the velocities and send them to the rear wheels controller
      [wr,wl] = calculate_velocities(v,w,alpha,P,Pinitial)
      
      # SEND HERE THE VELOCITIES TO THE CONTROLLER!!
      rightMotorSpeed = (60/(2*math.pi))*G*wr # [rpm]
      leftMotorSpeed = (60/(2*math.pi))*G*wl # [rpm]
      node.sdo[0x2002][2].raw = -round(rightMotorSpeed) # right motor
      node.sdo[0x2002][1].raw = round(leftMotorSpeed) # left motor

   # STOP THE WHEELCHAIR!!!!
   stopWheelchair(node)

######################################

def path_follow(v,w,node):
   # Grab data
   # Wheelchair initial position and caster wheels initial orientation
   odom.lock.acquire()
   px = odom.poseX
   py = odom.poseY
   pth = odom.poseTh # [rad]
   alpha_r = odom.RightCaster
   alpha_l = odom.LeftCaster
   odom.lock.release()
   Pinitial = [px,py,(180/math.pi)*pth] # Pose inicial [x,y,th] -> [[mm],[mm],[º]]
   alpha_initial = [alpha_r,alpha_l] # [[º],[º]]
   
   # Calculate the desired velocities for the rear wheels
   theta_r_dot = (1/Rright)*v+(L/(2*Rright))*w
   theta_l_dot = (1/left)*v-(L/(2*Rleft))*w
   
   epsilon = 0.0001
   if abs(w) < epsilon:
      # Calculate the coefficients of the line equation
      A = math.tan((math.pi/180)*Pinitial[2])
      B = -1
      C = -math.tan((math.pi/180)*Pinitial[2])*Pinitial[0]+Pinitial[1]
      
      # Calculate the steady state angles for the caster wheels
      alpha_ss = [0,0]
   else:
      # Calculate the curvature radius of the curve
      r = b*(theta_r_dot*Rright+theta_l_dot*Rleft)/(theta_r_dot*Rright-theta_l_dot*Rleft)
      
      # Calculate the position of the Instanteneous Center of Rotation (ICR)
      ICR = [Pinitial[0] - r*math.sin((math.pi/180)*Pinitial[2]), Pinitial[1] + r*math.cos((math.pi/180)*Pinitial[2])]
      
      # Calculate the steady state angles for the caster wheels
      alpha_ss = [(180/math.pi)*(math.atan2(Rl,r+Rf/2)), (180/math.pi)*(math.atan2(Rl,r-Rf/2))] # [[º],[º]]
   
   # CONTROL
   # Controller gains
   K = [0.0001,0.0001,1,0.0001]
   
   P = Pinitial
   alpha = alpha_initial
   
   int_D = 0 # Integral of D
   
   if abs(w) < epsilon:
      # Calculate the distance till the line
      D = abs(A*P[0]+B*P[1]+C)/math.sqrt(A**2+B**2)
      if P[1] < Pinitial[1]:
         D = -D
      
      # Calculate the difference between the desired heading and the current heading
      th = P[2]-Pinitial[2] # [º]
   else:
      # Calculate the distance till the curve
      D = r-math.sqrt((P[0]-ICR[0])**2 + (P[1]-ICR[1])**2)
      
      # Calculate the difference between the desired heading and the current heading
      phi = (180/math.pi)*math.atan2(P[1]-ICR[1],P[0]-ICR[0]) # [º]
      th = -(90+phi-P[2])
   
   int_D = int_D + D # Integral of D
   
   # Calculate initial Delta
   if ((alpha[0]-alpha_ss[0]) < -90 or (alpha[1]-alpha_ss[1]) < -90) or ((alpha[0]-alpha_ss[0]) > 90 or (alpha[1]-alpha_ss[1]) > 90):
      K[0] = 0
   else:
      K[0] = 0.0001
   Delta = K[0]*((alpha[0]-alpha_ss[0])+(alpha[1]-alpha_ss[1]))+K[1]*th+K[2]*D+K[3]*int_D
   
   # Calculate the initial velocities and send them to the rear wheels controller
   v = (theta_r_dot*Rright+theta_l_dot*Rleft)/2
   theta_r_dot = (v-Delta)/R # [rad/s] -> angular velocity of the right rear wheel
   theta_l_dot = (v+Delta)/R # [rad/s] -> angular velocity of the left rear wheel
   
   # SEND HERE THE VELOCITIES TO THE CONTROLLER!!
   rightMotorSpeed = (60/(2*math.pi))*G*theta_r_dot # [rpm]
   leftMotorSpeed = (60/(2*math.pi))*G*theta_l_dot # [rpm]
   node.sdo[0x2002][2].raw = -round(rightMotorSpeed) # right motor 
   node.sdo[0x2002][1].raw = round(leftMotorSpeed) # left motor
   time.sleep(0.5) # Wait till it reaches the velocity
   
   # Control loop
   while True:
      # Verify if a new command was issued
      if comm.event.is_set():
         return
      
      # Grab wheelchair position, caster wheels orientation and rear wheels velocities
      odom.lock.acquire()
      px = odom.poseX
      py = odom.poseY
      pth = odom.poseTh # [rad]
      alpha_r = odom.RightCaster
      alpha_l = odom.LeftCaster
      rightWheelAngularVelocity = odom.RightRotation
      leftWheelAngularVelocity = odom.LeftRotation
      odom.lock.release()
      P = [px,py,(180/math.pi)*pth] # Pose inicial [x,y,th] -> [[mm],[mm],[º]]
      alpha = [alpha_r,alpha_l] # [[º],[º]]
      WheelAngularVelocity = [rightWheelAngularVelocity,leftWheelAngularVelocity] # [[rad/s],[rad/s]]
      
      if abs(w) < epsilon:
         # Calculate the distance till the line
         D = abs(A*P[0]+B*P[1]+C)/math.sqrt(A**2+B**2)
         if P[1] < Pinitial[1]:
            D = -D
         
         # Calculate the difference between the desired heading and the current heading
         th = P[2]-Pinitial[2] # [º]
      else:
         # Calculate the distance till the curve
         D = r-math.sqrt((P[0]-ICR[0])**2 + (P[1]-ICR[1])**2)
         
         # Calculate the difference between the desired heading and the current heading
         phi = (180/math.pi)*math.atan2(P[1]-ICR[1],P[0]-ICR[0]) # [º]
         th = -(90+phi-P[2])
      
      # Calculate Delta
      if ((alpha[0]-alpha_ss[0]) < -90 or (alpha[1]-alpha_ss[1]) < -90) or ((alpha[0]-alpha_ss[0]) > 90 or (alpha[1]-alpha_ss[1]) > 90):
         K[0] = 0
      else:
         K[0] = 0.0001
      Delta = K[0]*((alpha[0]-alpha_ss[0])+(alpha[1]-alpha_ss[1]))+K[1]*th+K[2]*D+K[3]*int_D
      
      # Calculate the velocities and send them to the rear wheels controller
      v = (WheelAngularVelocity[0]*Rright+WheelAngularVelocity[1]*Rleft)/2
      theta_r_dot = (v-Delta)/R # [rad/s] -> angular velocity of the right rear wheel
      theta_l_dot = (v+Delta)/R # [rad/s] -> angular velocity of the left rear wheel
      
      # SEND HERE THE VELOCITIES TO THE CONTROLLER!!
      rightMotorSpeed = (60/(2*math.pi))*G*theta_r_dot # [rpm]
      leftMotorSpeed = (60/(2*math.pi))*G*theta_l_dot # [rpm]
      node.sdo[0x2002][2].raw = -round(rightMotorSpeed) # right motor
      node.sdo[0x2002][1].raw = round(leftMotorSpeed) # left motor
######################################

def controlThread(node):
   while True:
      comm.event.wait()
      command = comm.commands[0]  # stop(0), vel (1), rotvel (2), vel2 (3), heading (4), displ (5)
      par1 = comm.commands[1]     # vel, rot vel, vel right, heading, displ
      par2 = comm.commands[2]     # vel left
      comm.event.clear()
      # process the issued command

      # at each cycle of the command processing verify if a new command was issued
      # use comm.event.is_set() to check if the event was set
      # if true the command processing routine must return immediately
      # and the loop restarts from be beginning

      # for reading an odometry data in the command processin routines
      # odom.lock.acquire()
      # px = odom.poseX
      # py = odom.poseY
      # pth = odom.poseTh
      # odom.lock.release()

      # release dead man switch
      if par1 != 0 or par2 != 0:
         dead1Pin.write(0)
         dead2Pin.write(0)
         # zero encoder counters
         node.sdo[0x2003][1].raw = 0
         node.sdo[0x2003][2].raw = 0

      if command == 0:
         # STOP THE WHEELCHAIR
         stopWheelchair(node)
      elif command == 1:
         # SEND A LINEAR VELOCITY TO THE WHEELCHAIR
         v = par1
         w = 0
         path_follow(v,w,node)
      elif command == 2:
         # SEND AN ANGULAR VELOCITY TO THE WHEELCHAIR
         v = 0
         w = par1
         path_follow(v,w,node)
      elif command == 3:
         # SEND A LINEAR AND AN ANGULAR VELOCITY TO THE WHEELCHAIR
         vr = par1
         vl = par2
         v = (vr+vl)/2
         w = (vr-vl)/(2*b)
         path_follow(v,w,node)
      elif command == 4:
         # MAKE THE WHEELCHAIR TURN 'heading' DEGREES
         dth = par1
         heading(dth,node)
      elif command == 5:
         # MAKE THE WHEELCHAIR TRAVEL 'displ' MILLIMETERS
         ds = par1
         displ(ds,node)
      else:
         # COMMAND NOT ALLOWED
         print("Command not allowed")	



  

