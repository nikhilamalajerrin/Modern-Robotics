Kp=5
Ki=0

    initial_conf = np.array([-pi/4.0, -0.6, 0.3,0.0, 0.0, -pi/4, -0.3, 0.0,0.0, 0, 0.0, 0.0,0])
    "The configuration of the cube in the s frame in the inital position"
    Tsc_in = np.array([[1,0,0,1],[0,1,0,0],[0,0,1,0.025],[0,0,0,1]])
    "The configuration of the cube in the s frame in the final position"
    Tsc_goal = np.array([[0,1,0,0],[-1,0,0,-1],[0,0,1,0.025],[0,0,0,1]])