Kp=5
Ki=0

    initial_conf = np.array([-pi/4.0, -0.6, 0.3,0.0, 0.0, -pi/4.0, -0.3, 0.0,0.0, 0.0, 0.0, 0.0,0])
    "The configuration of the cube in the s frame in the inital position"
    Tsc_in = np.array([[m.cos(pi/4), m.sin(pi/4), 0.0, 0.5],[m.sin(-pi/4), m.cos(pi/4), 0.0, 0.5],[0.0, 0.0, 1.0, 0.025],[0.0, 0.0, 0.0, 1.0]])
    "The configuration of the cube in the s frame in the final position"
    Tsc_goal = np.array([[m.cos(pi), m.sin(pi), 0.0, 0.8],[m.sin(pi), m.cos(pi), 0.0, -0.6],[0.0, 0.0, 1.0, 0.025],[0.0, 0.0, 0.0, 1.0]])
