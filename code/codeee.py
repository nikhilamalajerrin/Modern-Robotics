import numpy as np
import math as m
import core as cr
import csv
import pandas as pd
import matplotlib.pyplot as plt

pi=m.pi 
#timestep
dt = 0.01

"Configurations for the reference trajectory"

th = 3*pi/4 #The end effector frame e is rotated about 3*pi/4 around y axis 
            #wrt to the cube frame c. 
"The configuration of the end effector when grasping in the cube frame"
Tce_grasp = np.array([[m.cos(th),0,m.sin(th),0.005],[0,1,0,0],[-m.sin(th),0,m.cos(th),0],[0,0,0,1]])
"The configuration of the end effector in the standoff segment in the cube frame"
Tce_standoff= np.array([[m.cos(th),0,m.sin(th),0],[0,1,0,0],[-m.sin(th),0,m.cos(th),0.3],[0,0,0,1]])
 
k = 1
duration = [1.5,1,0.5,0.5,3,0.5,0.5,1] #This will determine how long each segment will last
duration_tot=0
for t in duration:
    duration_tot = duration_tot + t #total duration
    
trajectories = []

#The following function computes the trajectory in each of the 8 segments
# and the trajectories of each segment together with the gripper state.
def TrajectoryGenerator(Tse_in,Tsc_in,Tsc_goal,Tce_grasp,Tce_standoff,k):
    
    "---------SEGMENT 1---------"
    N1 = duration[0]*k/0.01  
    Tse_standoff = np.dot(Tsc_in,Tce_standoff)
    Segment_1 = cr.CartesianTrajectory(Tse_in,Tse_standoff,duration[0],N1,5)
    trajectories.extend(Segment_1)
     
    "---------SEGMENT 2---------"
    N2 = duration[1]*k/0.01
    Tse_grasp = np.dot(Tsc_in,Tce_grasp)
    Segment_2 = cr.CartesianTrajectory(Tse_standoff,Tse_grasp,duration[1],N2,5)
    trajectories.extend(Segment_2)
     
    "---------SEGMENT 3---------"
    N3 = duration[2]*k/0.01  
    Segment_3 = cr.CartesianTrajectory(Tse_grasp,Tse_grasp,duration[2],N3,5)
    trajectories.extend(Segment_3)
     
    "---------SEGMENT 4---------"
    N4 = duration[3]*k/0.01
    Segment_4 = cr.CartesianTrajectory(Tse_grasp,Tse_standoff,duration[3],N4,5)
    trajectories.extend(Segment_4)
     
    "---------SEGMENT 5---------"
    N5 = duration[4]*k/0.01  
    Tse_standoff_2 = np.dot(Tsc_goal,Tce_standoff)
    Segment_5 = cr.CartesianTrajectory(Tse_standoff,Tse_standoff_2,duration[4],N5,5)
    trajectories.extend(Segment_5)
     
    "---------SEGMENT 6---------"
    N6 = duration[5]*k/0.01
    Tse_grasp = np.dot(Tsc_goal,Tce_grasp)
    Segment_6 = cr.CartesianTrajectory(Tse_standoff_2,Tse_grasp,duration[5],N6,5)
    trajectories.extend(Segment_6)
     
    "---------SEGMENT 7---------"
    N7 = duration[6]*k/0.01  
    Segment_7 = cr.CartesianTrajectory(Tse_grasp,Tse_grasp,duration[6],N7,5)
    trajectories.extend(Segment_7)
     
    "---------SEGMENT 8---------"
    N8 = duration[7]*k/0.01  
    Segment_8 = cr.CartesianTrajectory(Tse_grasp,Tse_standoff_2,duration[7],N8,5)
    trajectories.extend(Segment_8)
     
    return trajectories,Segment_1,Segment_2,Segment_3,Segment_4,Segment_5,Segment_6,Segment_7,Segment_8

"Function to compute next state"
def NextState(current,speeds,dt,max_speed=10000):
    current.tolist()
    speeds.tolist()
    l = 0.235
    w = 0.15
    r = 0.0475    
    
    #the current arm and wheel angles
    old_arm_angles = current_conf[3:8]
    old_wheel_angles = current_conf[8:12]
    wheel_speed = speed[0:4]
    joint_speed = speed[4:9]
    
    #compute the new arm and wheel angles
    arm_angles = old_arm_angles + joint_speed*dt 
    wheel_angles = old_wheel_angles + wheel_speed*dt
    
    #chassis phi
    phi = current_conf[0]   
    deltaTheta = wheel_speed*dt
    F=np.array([[-r/(4*(l+w)),r/(4*(l+w)),r/(4*(l+w)),-r/(4*(l+w))],[r/4,r/4,r/4,r/4],[-r/4,r/4,-r/4,r/4]])
    Vb =np.dot(F,deltaTheta.T)
    Vb6 = np.array([0,0,Vb[0],Vb[1],Vb[2],0]).T
    q_old = np.array([current_conf[0],current_conf[1],current_conf[2]]).T
    if Vb6[2]==0:
        dqb = np.array([0,Vb6[3],Vb6[4]]).T
    else:
        dqb  = np.array([Vb6[2],(Vb6[3]*m.sin(Vb6[2])+Vb6[4]*(m.cos(Vb6[2])-1))/Vb6[2],(Vb6[4]*m.sin(Vb6[2])+Vb6[3]*(1-m.cos(Vb6[2])))/Vb6[2]]).T
    delta_q = np.array([[1,0,0],[0,m.cos(phi),-m.sin(phi)],[0,m.sin(phi),m.cos(phi)]])
    #the new configuration
    q = q_old + np.dot(delta_q,dqb)
    new_state = np.array([q[0],q[1],q[2],arm_angles[0],arm_angles[1],arm_angles[2],arm_angles[3],arm_angles[4],wheel_angles[0],wheel_angles[1],wheel_angles[2],wheel_angles[3]])    
    return new_state

"Function to compute feedback control"
def FeedbackControl(X,Xd,Xd_next,actual_se_config,Kp,Ki,dt,int_total_error):
    l = 0.235
    w = 0.15
    r = 0.0475
    F6 = np.array([[0,0,0,0],[0,0,0,0],[(r/4)*(-1/(l+w)),(r/4)*(1/(l+w)),(r/4)*(1/(l+w)),(r/4)*(-1/(l+w))],[(r/4),(r/4),(r/4),(r/4)],[(-r/4),(r/4),(-r/4),(r/4)],[0,0,0,0]])
    Tb0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])
    M0e = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])    
    Blist = np.array([[0,0,1,0,0.033,0],[0,-1,0,-0.5076,0,0],[0,-1,0,-0.3526,0,0],[0,-1,0,-0.2176,0,0],[0,0,1,0,0,0]]).T
    thetalist = np.array([actual_se_config[3:8]]).T
    "Import several functions from the given library named core in the directory results"
    T0e = cr.FKinBody(M0e, Blist, thetalist) 
    Jbase = np.dot(cr.Adjoint(np.dot(np.linalg.inv(T0e),np.linalg.inv(Tb0))),F6)
    Jarm = cr.JacobianBody(Blist,thetalist) 
    Je = np.hstack((Jbase, Jarm))
    Je_pseudo = np.linalg.pinv(Je)
      
    Vd = cr.se3ToVec(cr.MatrixLog6(np.dot(np.linalg.inv(Xd),Xd_next))*(1/dt))
    Xerr = cr.se3ToVec(cr.MatrixLog6(np.dot(np.linalg.inv(X),Xd)))
    adjoint = cr.Adjoint(np.dot(np.linalg.inv(X),Xd))
    total_error = np.add(Xerr, int_total_error)
    "Control law"
    V = np.dot(adjoint,Vd) + np.dot(Kp,Xerr) + np.dot(Ki,total_error)
    "Controls: velocities of wheels and arms"
    u_theta = np.dot(Je_pseudo,V)
    return u_theta,Xerr,total_error

"The following function computes Tse. This matrix is the first input in the Feedback control"
"later in the code. Given a known configuration we compute Tse."
def compute_Tse(conf):
    B = np.array([[0,0,1,0,0.033,0],[0,-1,0,-0.5076,0,0],[0,-1,0,-0.3526,0,0],[0,-1,0,-0.2176,0,0],[0,0,1,0,0,0]]).T
    M0e = np.array([[1,0,0,0.033],[0,1,0,0],[0,0,1,0.6546],[0,0,0,1]])    

    v1 = np.matrix(cr.MatrixExp6(cr.VecTose3(np.dot(conf[3], B[:,0]))))
    v2 = np.matrix(cr.MatrixExp6(cr.VecTose3(np.dot(conf[4], B[:,1]))))
    v3 = np.matrix(cr.MatrixExp6(cr.VecTose3(np.dot(conf[5], B[:,2]))))
    v4 = np.matrix(cr.MatrixExp6(cr.VecTose3(np.dot(conf[6], B[:,3]))))
    v5 = np.matrix(cr.MatrixExp6(cr.VecTose3(np.dot(conf[7], B[:,4]))))

    Tb0 = np.array([[1,0,0,0.1662],[0,1,0,0],[0,0,1,0.0026],[0,0,0,1]])
    T0e = (M0e*v1*v2*v3*v4*v5).tolist()
    Tsb = np.matrix([[m.cos(conf[0]), -1 * m.sin(conf[0]), 0, conf[1]],[m.sin(conf[0]), m.cos(conf[0]), 0, conf[2]],[0, 0, 1, 0.0963],[0, 0, 0, 1]])
    Tse = (Tsb*Tb0*T0e).tolist()
    return Tse

"The following function corresponds to the configurations that provide the best output"
def best():
    initial_conf = np.array([-pi/4.0, -0.6, 0.3,0.0, 0.0, -pi/4, -0.3, 0.0,0.0, 0, 0.0, 0.0,0])
    "The configuration of the cube in the s frame in the inital position"
    Tsc_in = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
    "The configuration of the cube in the s frame in the final position"
    Tsc_goal = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])
    Kp=np.dot(np.diag((1, 1, 1, 1, 1, 1)), 5)
    Ki=np.dot(np.diag((1, 1, 1, 1, 1, 1)), 0)
    return initial_conf,Tsc_in,Tsc_goal,Kp,Ki

"The following function corresponds to the configurations that provide the overshoot output"
def overshoot():
    initial_conf = np.array([-pi/4.0, -0.6, 0.3,0.0, 0.0, -pi/4.0, 0.3, 0.0,0.0, 0, 0.0, 0.0,0])
 
    "The configuration of the cube in the s frame in the inital position"
    Tsc_in = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
    "The configuration of the cube in the s frame in the final position"
    Tsc_goal = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])
    Kp=np.dot(np.diag((1, 1, 1, 1, 1, 1)), 5)
    Ki=np.dot(np.diag((1, 1, 1, 1, 1, 1)), 0.2)
    return initial_conf,Tsc_in,Tsc_goal,Kp,Ki

"The following function corresponds to the configurations that provide the new block output"
def new_task():
    initial_conf = np.array([-pi/4.0, -0.6, 0.3,0.0, 0.0, -pi/4.0, -0.3, 0.0,0.0, 0.0, 0.0, 0.0,0])
    "The configuration of the cube in the s frame in the inital position"
    Tsc_in = np.array([[m.cos(pi/4), m.sin(pi/4), 0.0, 0.5],[m.sin(-pi/4), m.cos(pi/4), 0.0, 0.5],[0.0, 0.0, 1.0, 0.025],[0.0, 0.0, 0.0, 1.0]])
    "The configuration of the cube in the s frame in the final position"
    Tsc_goal = np.array([[m.cos(pi), m.sin(pi), 0.0, 0.8],[m.sin(pi), m.cos(pi), 0.0, -0.6],[0.0, 0.0, 1.0, 0.025],[0.0, 0.0, 0.0, 1.0]])
    Kp=np.dot(np.diag((1, 1, 1, 1, 1, 1)), 5)
    Ki=np.dot(np.diag((1, 1, 1, 1, 1, 1)), 0)
    return initial_conf,Tsc_in,Tsc_goal,Kp,Ki

"The flollowing three lines call one of the configurations for each of the three cases"
"best,overshoot,new task. To produce the outcome for each of these cases please comment or uncomment correspondingely"
#initial_conf,Tsc_in,Tsc_goal,Kp,Ki = best()
initial_conf,Tsc_in,Tsc_goal,Kp,Ki = overshoot()
#initial_conf,Tsc_in,Tsc_goal,Kp,Ki = new_task()

error = []
conf = []
#set initial configuration to current configuraiton
current_conf = initial_conf
# a is a matrix that will help us append every kth configuration to a csv file
a = current_conf
#the reference Tse
Tse_in_reference = np.array([[0.0, 0.0, 1.0, 0.0],[0.0, 1.0, 0.0, 0.0],[-1.0, 0.0, 0.0, 0.5],[0.0, 0.0, 0.0, 1.0]])
#given the initial configuraiton we use the compute_Tse function to compute Tse
Tse_in_real = compute_Tse(initial_conf)

Tse_r = Tse_in_real

"Generates the trajectories and each segment 1 through 8 with a value 0 or 1 for the gripper state"
trajectories,s1,s2,s3,s4,s5,s6,s7,s8 = TrajectoryGenerator(Tse_in_reference,Tsc_in,Tsc_goal,Tce_grasp,Tce_standoff,k)
 
total_error = 0
for t in range(len(trajectories)-1):
    #assigns 0 gripper state for segments 1 and 2
    if t<=len(s1)+len(s2):
        con = [a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8],a[9],a[10],a[11],0]
    #assigns 1 gripper state for segments 3 to 6
    elif (t>len(s1)+len(s2)) & (t<=len(s1)+len(s2)+len(s3)+len(s4)+len(s5)+len(s6)):
        con = [a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8],a[9],a[10],a[11],1]
    #assigns 0 gripper state for segments 7 and 8
    else:
        con = [a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],a[8],a[9],a[10],a[11],0]
    conf.append(con)
    Xd = trajectories[t]
    Xd_next = trajectories[t+1]
    controls,x_error, xer = FeedbackControl(Tse_r,Xd, Xd_next,current_conf ,Kp, Ki, dt,total_error)
    #assigns the velocities of the wheels and joints to an array
    speed = np.array([controls[0],controls[1],controls[2],controls[3],controls[4],controls[5],controls[6],controls[7],controls[8]])

    new_conf = NextState(current_conf,speed,dt)
    Tse_r = compute_Tse(current_conf)
    current_conf = new_conf
    a = current_conf
    total_error = xer
    error.append(x_error)


print('Generating animation csv file.')
with open('Configuration.csv', 'w') as conf_csv:
    writer = csv.writer(conf_csv)
    for i in range(len(conf)):
        current_conf= conf[i]
        writer.writerow(current_conf)
print('Writing error plot data.')
with open('error.csv', 'w') as conf_csv:
    writer = csv.writer(conf_csv)
    for i in range(len(error)):
        current_conf= error[i]
        writer.writerow(current_conf)
print('Done.')

