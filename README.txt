The end effector frame e is rotated about 3*pi/4 around y axis wrt to the cube frame c.

Initially I define the three functions given by Capstone 1,2 and 3.
The  TrajectoryGenerator reurns the trajectory in each of the 8 segments
and the trajectories of each segment together with the gripper state.
The NextState function returns the next state. I followed Chapter 13 of the book.
The FeedbackControl function is based on chapters 13,3 and 4. I used some functions from
the library from chapter 3 and 4 to compute Jbase and Jarm. It returns the wheel and joint
velocities noted later as controls and the errors.
The function compute_Tse returns the matrix Tse. This matrix is the first input in the FeedbackControl
later in the code. Given a known configuration it returns Tse.
The functions best,overshoot and new_task return the configurations corresponding to the best output to an output 
with overshoot and to an output that the block's position is different than the initial.
We then call the TrajectoryGenerator function and generate the trajectories and each segments 1 
through 8 with a value 0 or 1 for the gripper state.
Then we enter the for loop to generate a csv file with the configurations. The if statement 
help us assign 0 or 1 gripper state for each configuration.