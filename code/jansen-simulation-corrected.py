import numpy as np

#-------------------------------#
#                               #
#       system parameters       #
#                               #
#-------------------------------#
# masses, lengths, etc.

ell_0 = 0.057168439518905365
ell_1 = 0.19056146506301788
ell_2 = 0.21266659501032797
ell_3 = 0.1501624344696581
ell_4 = 0.25039776509280554
ell_5 = 0.18675023576175753
ell_6 = 0.23591509374801614
ell_7 = 0.13987211535625513
ell_8 = 0.14978131153953206
ell_9 = 0.15816601600230484
ell_10 = 0.15283029498054035
x_pin = 0.14482671344789358
y_pin = -0.02972758854983079

# dimensions taken from Capstone, sufficiently strong to carry 90kg robot
tube_density    = 2700      # kg/m^3, alu 6061
tube_outer_diam = 0.044     # 44.0mm
tube_inner_diam = 0.0408    # 40.8mm, tube thickness of 3.175mm
A = (np.pi * tube_outer_diam**2) - (np.pi * tube_inner_diam**2)

# ANYdrive doesn't release mass data. max output torque is 40Nm, so about a Hebi robotics X8-16
m_motor = 0.49              # kg

m0 = ell_0 * A * tube_density
m1 = ell_1 * A * tube_density
m2 = ell_2 * A * tube_density
m3 = ell_3 * A * tube_density
m4 = ell_4 * A * tube_density
m5 = ell_5 * A * tube_density
m6 = ell_6 * A * tube_density
m7 = ell_7 * A * tube_density
m8 = ell_8 * A * tube_density
m9 = ell_9 * A * tube_density
m10= ell_10 * A * tube_density
m_torso = 30 / 3 # only carries 1/3 of the weight at a time

# foot contact force
g = 9.806
Fe = (m0+m1+m2+m3+m4+m5+m6+m7+m8+m9+m10+m_torso)*g

#-------------------------------#
#                               #
#       stride length test      #
#                               #
#-------------------------------#
# before running the simulation, we need to know what positions of theta the foot is in contact with the ground, and which it is not

steps = 100000              # how many steps in the simulation
margin = 0.0035             # how close is considered foot contact, currently 1% of foot height
# np.empty(i) returns an empty array of size i
theta = np.empty(steps)
x_5 = np.empty(steps)
y_5 = np.empty(steps)
# saving pertinent values
i_begin = -1;                # index at which ground contact is made
i_end   = -1;                # index at which ground contact is made
# going from 2pi to 0 gives clockwise rotation
for j in range(steps, 0, -1):
    # j gives the proper value of theta (beginning at 2pi at steps), i is the index in the array (beginning at 0)
    i = steps - j
    theta[i] = 2*np.pi * (j/steps)
    # equations for x5 and y5
    x_5[i] = 0.00010890777981677994*theta[i]**7 - 0.0023613255812160085*theta[i]**6 + 0.019821300619349815*theta[i]**5 - 0.080086038442003765*theta[i]**4 + 0.14477794740074396*theta[i]**3 - 0.023229035399887493*theta[i]**2 - 0.17501397224229095*theta[i] + 0.13052110438492678
    y_5[i] = -9.1060640760102596e-5*theta[i]**7 + 0.0019414992080475665*theta[i]**6 - 0.016468072933240097*theta[i]**5 + 0.071557923036943416*theta[i]**4 - 0.17110944027246222*theta[i]**3 + 0.22868793144811714*theta[i]**2 - 0.17387938800955383*theta[i] + -0.27720874027616921
    # foot contact is begun when y_5 drops under margin (and has not previously dropped under)
    if (y_5[i] <= -0.35+margin and i_begin == -1):
        i_begin = i
    # foot contact has ended when y_5 rises above the margin (and foot contact has previously been recorded)
    elif (y_5[i] >= -0.35+margin and i_begin != -1 and i_end == -1):
        i_end = i

stride_length = x_5[i_begin]-x_5[i_end]

theta_contact = theta[i_begin]
theta_liftoff = theta[i_end]

#-------------------------------#
#                               #
#  Find air and contact time    #
#                               #
#-------------------------------#
# after performing this with a step size of 100 000, found:
# stride length of 0.221278m
# according to 'TOWARD COMBINING SPEED, EFFICIENCY, VERSATILITY, AND ROBUSTNESS IN AN AUTONOMOUS QUADRUPED'
# a leg's stride occupies three quarters of the time to complete an entire cycle
# so 0.221278 * 1 1/3
full_cycle_distance = stride_length * (1 + (1/3))
# robot walking speed is 0.3m/s
time_cycle      = full_cycle_distance / 0.3
# the time spent with ground contact should present 3/4 of the total time
time_ground   = time_cycle * (3/4)
time_air         = time_cycle * (1/4)

#print('target cycle time: ',time_cycle)
#print('target ground time: ',time_ground)
#print('target air time: ',time_air)

# find the velocities needed during contact and flight phases
ground_velocity    = abs(theta[i_begin] - theta[i_end]) / time_ground
flight_velocity     = (2*np.pi - abs(theta[i_begin] - theta[i_end])) / time_air

#-------------------------------#
#                               #
#       SIMULATION SETUP        #
#                               #
#-------------------------------#
step_size   = time_cycle / steps
time        = np.empty(steps)
torque      = np.empty(steps)
# reactions at joints
R_4         = np.empty(steps)
R_5         = np.empty(steps)
R_3         = np.empty(steps)
R_7         = np.empty(steps)
R_6         = np.empty(steps)
R_8         = np.empty(steps)
R_2         = np.empty(steps)
R_10        = np.empty(steps)
R_1         = np.empty(steps)
R_9         = np.empty(steps)
# actuator position, velocities, accelerations
theta_d     = np.empty(steps)
theta_dd    = np.empty(steps)
theta       = np.empty(steps)
# joint positions
x_0         = np.empty(steps)
y_0         = np.empty(steps)
x_1         = np.empty(steps)
y_1         = np.empty(steps)
x_2         = np.empty(steps)
y_2         = np.empty(steps)
x_3         = np.empty(steps)
y_3         = np.empty(steps)
x_4         = np.empty(steps)
y_4         = np.empty(steps)
x_5         = np.empty(steps)
y_5         = np.empty(steps)
# joint velocities
x_0d        = np.empty(steps)
y_0d        = np.empty(steps)
x_1d        = np.empty(steps)
y_1d        = np.empty(steps)
x_2d        = np.empty(steps)
y_2d        = np.empty(steps)
x_3d        = np.empty(steps)
y_3d        = np.empty(steps)
x_4d        = np.empty(steps)
y_4d        = np.empty(steps)
x_5d        = np.empty(steps)
y_5d        = np.empty(steps)
# joint accelerations
x_0dd       = np.empty(steps)
y_0dd       = np.empty(steps)
x_1dd       = np.empty(steps)
y_1dd       = np.empty(steps)
x_2dd       = np.empty(steps)
y_2dd       = np.empty(steps)
x_3dd       = np.empty(steps)
y_3dd       = np.empty(steps)
x_4dd       = np.empty(steps)
y_4dd       = np.empty(steps)
x_5dd       = np.empty(steps)
y_5dd       = np.empty(steps)

# initial configuration
time[0]     = 0
torque[0]   = 0
theta_d[0]  = ground_velocity
theta_dd[0] = 0
theta[0]    = theta_contact
x_0[0] = 0.057168439518905365*np.cos(theta[0])
y_0[0] = 0.057168439518905365*np.sin(theta[0])
x_1[0] = -3.9444093759977156e-5*theta[0]**7 + 0.00079651887729376028*theta[0]**6 - 0.0060982936002913785*theta[0]**5 + 0.021142731056925997*theta[0]**4 - 0.02318575947860551*theta[0]**3 - 0.038591424819869651*theta[0]**2 + 0.044786929093735252*theta[0] + 0.21136233094291412
y_1[0] = 9.7696232078669311e-7*theta[0]**7 - 1.8729927370066667e-5*theta[0]**6 - 0.00026445111392442845*theta[0]**5 + 0.0057094233361158426*theta[0]**4 - 0.032045978719843708*theta[0]**3 + 0.067754275638143499*theta[0]**2 - 0.04185448571003534*theta[0] + 0.1154854870501546
x_2[0] = -1.5385832735394109e-6*theta[0]**7 + 3.2032304984172107e-5*theta[0]**6 - 0.0006385016212455991*theta[0]**5 + 0.0068347574586696347*theta[0]**4 - 0.032357378267202482*theta[0]**3 + 0.062902817699119776*theta[0]**2 - 0.037540189632133958*theta[0] + 0.28902787219277493
y_2[0] = 3.8094080236292624e-5*theta[0]**7 - 0.00076919443006912903*theta[0]**6 + 0.0058634434608907191*theta[0]**5 - 0.020027109111658687*theta[0]**4 + 0.020340759774849764*theta[0]**3 + 0.041471458856325326*theta[0]**2 - 0.045816462252947519*theta[0] + -0.084750063060428746
x_3[0] = 9.0035479393536157e-6*theta[0]**7 - 0.00024218154094739412*theta[0]**6 + 0.0022357559837752339*theta[0]**5 - 0.0086561417963357933*theta[0]**4 + 0.00653020377239724*theta[0]**3 + 0.069679590351408541*theta[0]**2 - 0.21561570380817119*theta[0] + 0.25551004843366304
y_3[0] = -6.7219034741546779e-5*theta[0]**7 + 0.0014408981619693983*theta[0]**6 - 0.012922503096705222*theta[0]**5 + 0.063138019057105921*theta[0]**4 - 0.17760590288224987*theta[0]**3 + 0.27072144266689652*theta[0]**2 - 0.18522866882411315*theta[0] + -0.13424665432811231
x_4[0] = 1.4806298276945396e-5*theta[0]**7 - 0.00036148058400403086*theta[0]**6 + 0.0027756327090590407*theta[0]**5 - 0.0064252278603851394*theta[0]**4 - 0.01464409994447291*theta[0]**3 + 0.11176713219788105*theta[0]**2 - 0.22881679392451476*theta[0] + 0.37603897613105641
y_4[0] = 9.4892499425732184e-6*theta[0]**7 - 0.00018561273919809877*theta[0]**6 + 0.00050289851684656901*theta[0]**5 + 0.0093142963376638549*theta[0]**4 - 0.076132571589480683*theta[0]**3 + 0.20698703848294936*theta[0]**2 - 0.15654508872664169*theta[0] + -0.20975554364335181
x_5[0] = 0.00010890777981677994*theta[0]**7 - 0.0023613255812160085*theta[0]**6 + 0.019821300619349815*theta[0]**5 - 0.080086038442003765*theta[0]**4 + 0.14477794740074396*theta[0]**3 - 0.023229035399887493*theta[0]**2 - 0.17501397224229095*theta[0] + 0.13052110438492678
y_5[0] = -9.1060640760102596e-5*theta[0]**7 + 0.0019414992080475665*theta[0]**6 - 0.016468072933240097*theta[0]**5 + 0.071557923036943416*theta[0]**4 - 0.17110944027246222*theta[0]**3 + 0.22868793144811714*theta[0]**2 - 0.17387938800955383*theta[0] + -0.27720874027616921
x_0d[0]  = 0
y_0d[0]  = 0
x_1d[0]  = 0
y_1d[0]  = 0
x_2d[0]  = 0
y_2d[0]  = 0
x_3d[0]  = 0
y_3d[0]  = 0
x_4d[0]  = 0
y_4d[0]  = 0
x_5d[0]  = 0
y_5d[0]  = 0
x_0dd[0]  = 0
y_0dd[0]  = 0
x_1dd[0]  = 0
y_1dd[0]  = 0
x_2dd[0]  = 0
y_2dd[0]  = 0
x_3dd[0]  = 0
y_3dd[0]  = 0
x_4dd[0]  = 0
y_4dd[0]  = 0
x_5dd[0]  = 0
y_5dd[0]  = 0
"""
R_4[0] = 0
R_5[0] = 0
R_3[0] = 0
R_7[0] = 0
R_6[0] = 0
R_8[0] = 0
R_2[0] = 0
R_10   = 0
R_1[0] = 0
R_9[0] = 0
"""

nanfound = False
#-------------------------------#
#                               #
#       SIMULATION TIME!        #
#                               #
#-------------------------------#
for i in range(1,steps,1):
    time[i] = time[i-1] + step_size
    # 1. calculate new position of theta
    if (y_5[i-1] <= -0.35+margin):
        # on the ground, clockwise
        theta[i] = theta[i-1] - ground_velocity * step_size
        theta_d[i] = ground_velocity
    else:
        # in the air, clockwise
        theta[i] = theta[i-1] - flight_velocity * step_size
        theta_d[i] = flight_velocity
    # we'll let theta become negative, as it shouldn't change the functionality of the program and helps avoid the "massive change in theta" problem
    theta_dd[i] = (theta[i] - theta[i-1] - theta_d[i]*step_size) / (step_size**2)
    # positions
    x_0[i] = 0.057168439518905365*np.cos(theta[i])
    y_0[i] = 0.057168439518905365*np.sin(theta[i])
    x_1[i] = -3.9444093759977156e-5*theta[i]**7 + 0.00079651887729376028*theta[i]**6 - 0.0060982936002913785*theta[i]**5 + 0.021142731056925997*theta[i]**4 - 0.02318575947860551*theta[i]**3 - 0.038591424819869651*theta[i]**2 + 0.044786929093735252*theta[i] + 0.21136233094291412
    y_1[i] = 9.7696232078669311e-7*theta[i]**7 - 1.8729927370066667e-5*theta[i]**6 - 0.00026445111392442845*theta[i]**5 + 0.0057094233361158426*theta[i]**4 - 0.032045978719843708*theta[i]**3 + 0.067754275638143499*theta[i]**2 - 0.04185448571003534*theta[i] + 0.1154854870501546
    x_2[i] = -1.5385832735394109e-6*theta[i]**7 + 3.2032304984172107e-5*theta[i]**6 - 0.0006385016212455991*theta[i]**5 + 0.0068347574586696347*theta[i]**4 - 0.032357378267202482*theta[i]**3 + 0.062902817699119776*theta[i]**2 - 0.037540189632133958*theta[i] + 0.28902787219277493
    y_2[i] = 3.8094080236292624e-5*theta[i]**7 - 0.00076919443006912903*theta[i]**6 + 0.0058634434608907191*theta[i]**5 - 0.020027109111658687*theta[i]**4 + 0.020340759774849764*theta[i]**3 + 0.041471458856325326*theta[i]**2 - 0.045816462252947519*theta[i] + -0.084750063060428746
    x_3[i] = 9.0035479393536157e-6*theta[i]**7 - 0.00024218154094739412*theta[i]**6 + 0.0022357559837752339*theta[i]**5 - 0.0086561417963357933*theta[i]**4 + 0.00653020377239724*theta[i]**3 + 0.069679590351408541*theta[i]**2 - 0.21561570380817119*theta[i] + 0.25551004843366304
    y_3[i] = -6.7219034741546779e-5*theta[i]**7 + 0.0014408981619693983*theta[i]**6 - 0.012922503096705222*theta[i]**5 + 0.063138019057105921*theta[i]**4 - 0.17760590288224987*theta[i]**3 + 0.27072144266689652*theta[i]**2 - 0.18522866882411315*theta[i] + -0.13424665432811231
    x_4[i] = 1.4806298276945396e-5*theta[i]**7 - 0.00036148058400403086*theta[i]**6 + 0.0027756327090590407*theta[i]**5 - 0.0064252278603851394*theta[i]**4 - 0.01464409994447291*theta[i]**3 + 0.11176713219788105*theta[i]**2 - 0.22881679392451476*theta[i] + 0.37603897613105641
    y_4[i] = 9.4892499425732184e-6*theta[i]**7 - 0.00018561273919809877*theta[i]**6 + 0.00050289851684656901*theta[i]**5 + 0.0093142963376638549*theta[i]**4 - 0.076132571589480683*theta[i]**3 + 0.20698703848294936*theta[i]**2 - 0.15654508872664169*theta[i] + -0.20975554364335181
    x_5[i] = 0.00010890777981677994*theta[i]**7 - 0.0023613255812160085*theta[i]**6 + 0.019821300619349815*theta[i]**5 - 0.080086038442003765*theta[i]**4 + 0.14477794740074396*theta[i]**3 - 0.023229035399887493*theta[i]**2 - 0.17501397224229095*theta[i] + 0.13052110438492678
    y_5[i] = -9.1060640760102596e-5*theta[i]**7 + 0.0019414992080475665*theta[i]**6 - 0.016468072933240097*theta[i]**5 + 0.071557923036943416*theta[i]**4 - 0.17110944027246222*theta[i]**3 + 0.22868793144811714*theta[i]**2 - 0.17387938800955383*theta[i] + -0.27720874027616921
    # velocities
    x_0d[i] = (x_0[i] - x_0[i-1]) / step_size
    y_0d[i] = (y_0[i] - y_0[i-1]) / step_size
    x_1d[i] = (x_1[i] - x_1[i-1]) / step_size
    y_1d[i] = (y_1[i] - y_1[i-1]) / step_size
    x_2d[i] = (x_2[i] - x_2[i-1]) / step_size
    y_2d[i] = (y_2[i] - y_2[i-1]) / step_size
    x_3d[i] = (x_3[i] - x_3[i-1]) / step_size
    y_3d[i] = (y_3[i] - y_3[i-1]) / step_size
    x_4d[i] = (x_4[i] - x_4[i-1]) / step_size
    y_4d[i] = (y_4[i] - y_4[i-1]) / step_size
    x_5d[i] = (x_5[i] - x_5[i-1]) / step_size
    y_5d[i] = (y_5[i] - y_5[i-1]) / step_size
    # accelerations
    x_0dd[i] = (x_0[i] - x_0[i-1] - x_0d[i]*step_size) / (step_size**2)
    y_0dd[i] = (y_0[i] - y_0[i-1] - y_0d[i]*step_size) / (step_size**2)
    x_1dd[i] = (x_1[i] - x_1[i-1] - x_1d[i]*step_size) / (step_size**2)
    y_2dd[i] = (y_2[i] - y_2[i-1] - y_2d[i]*step_size) / (step_size**2)
    x_3dd[i] = (x_3[i] - x_3[i-1] - x_3d[i]*step_size) / (step_size**2)
    y_3dd[i] = (y_3[i] - y_3[i-1] - y_3d[i]*step_size) / (step_size**2)
    x_4dd[i] = (x_4[i] - x_4[i-1] - x_4d[i]*step_size) / (step_size**2)
    y_4dd[i] = (y_4[i] - y_4[i-1] - y_4d[i]*step_size) / (step_size**2)
    x_5dd[i] = (x_5[i] - x_5[i-1] - x_5d[i]*step_size) / (step_size**2)
    y_5dd[i] = (y_5[i] - y_5[i-1] - y_5d[i]*step_size) / (step_size**2)
    
    # joint angles, taken from onenote file with free body diagrams
    theta5 = np.arccos((x_5[i] - x_3[i]) / ell_5)
    theta4 = np.arccos((x_5[i] - x_4[i]) / ell_4)
    theta7 = np.arccos((x_4[i] - x_3[i]) / ell_7)
    theta3 = np.arccos((x_4[i] - x_2[i]) / ell_3)
    theta6 = np.arccos((x_3[i] - x_0[i]) / ell_6)
    theta8 = np.arccos((x_3[i] - x_pin)  / ell_8)
    theta10= np.arccos((x_2[i] - x_pin)  / ell_10)
    theta2 = np.arccos((x_2[i] - x_1[i]) / ell_2)
    theta1 = np.arccos((x_1[i] - x_0[i]) / ell_1)
    theta9 = np.arccos((x_1[i] - x_pin)  / ell_9)
    
    if (np.isnan(theta5) and nanfound == False):
        nanfound = True
        print('x5: {0}, x3: {1}, x5-x3 = {2}, ell5 = {3}'.format(x_5[i],x_3[i],x_5[i]-x_3[i],ell_5))
        """print('x0: ',x_0[1])
        print('y0: ',y_0[1])
        print('x1: ',x_1[1])
        print('y1: ',y_1[1])
        print('x2: ',x_2[1])
        print('y2: ',y_2[1])
        print('x3: ',x_3[1])
        print('y3: ',y_3[1])
        print('x4: ',x_4[1])
        print('y4: ',y_4[1])
        print('x5: ',x_5[1])
        print('y5: ',y_5[1])"""
    
    if (y_5[i] <= -0.35+margin):
        # foot contact phase, include contact for Fe
        R5 = ((m4+m5)*(y_5dd[i] + g - x_5dd[i]*np.tan(theta4)) - Fe) / (np.sin(theta5) - np.cos(theta5)*np.tan(theta4))
    else:
        # flight phase, no contact force Fe
        R5 = ((m4+m5)*(y_5dd[i] + g - x_5dd[i]*np.tan(theta4))) / (np.sin(theta5) - np.cos(theta5)*np.tan(theta4))
    
    # reaction forces at all joints
    R4 = ((m4+m5) - R5*np.cos(theta5)) / (np.cos(theta4))
    R7 = ((m3+m7)*(y_4dd[i] + g - x_4dd[i]*np.tan(theta3)) + R4*(np.sin(theta4) - np.cos(theta4)*np.tan(theta3))) / (np.sin(theta7) - np.cos(theta7)*np.tan(theta3))
    R3 = ((m3+m7)*x_4dd[i] - R7*np.cos(theta7) + R4*np.cos(theta4)) / (np.cos(theta3))
    R8 = ((m6+m8)*(y_3dd[i] + g - x_3dd[i]*np.tan(theta3)) + R5*(np.sin(theta5) - np.cos(theta5)*np.tan(theta6)) + R7*(np.sin(theta6)-np.cos(theta7)*np.tan(theta7))) / (np.sin(theta8) - np.cos(theta8)*np.tan(theta6))
    R6 = ((m6+m8)*x_3dd[i] - R8*np.cos(theta8) + R5*np.cos(theta5) + R7*np.cos(theta7)) / (np.cos(theta6))
    R10= ((m2+m10)*(y_2dd[i] + g - x_2dd[i]*np.tan(theta2)) + R3*(np.sin(theta3) - np.cos(theta3)*np.tan(theta2))) / (np.sin(theta10) - np.cos(theta10)*np.tan(theta2))
    R2 = ((m2+m10)*x_2dd[i] - R10*np.cos(theta10) + R3*np.cos(theta3)) / (np.cos(theta2))
    R9 = ((m1+m9)*(y_1dd[i] + g - x_1dd[i]*np.tan(theta1)) + R2*(np.sin(theta2) - np.cos(theta2)*np.tan(theta1))) / (np.sin(theta9) - np.cos(theta9)*np.tan(theta1))
    R1 = ((m1+m9)*x_1dd[i] - R9*np.cos(theta9) + R2*np.cos(theta2)) / (np.cos(theta1))
    
    # motor torque
    torque[i] = (-R1*np.cos(theta1) - R6*np.cos(theta6)) * ell_0 * np.sin(theta[i]) + (-R1*np.sin(theta1) - R6*np.sin(theta6) - m0*g) * ell_0 * np.cos(theta[i])
    
#-------------------------------#
#                               #
#       export torques          #
#                               #
#-------------------------------#
# export to .csv for analysis

np.savetxt('jansen_simulation_corrected.csv',np.transpose(np.array([time,theta,theta_d,theta_dd,x_5,y_5,torque])))