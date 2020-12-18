import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class Vehicle():
    def __init__(self):
 
        # ==================================
        #  Parameters
        # ==================================
    
        #Throttle to engine torque
        self.a_0 = 400
        self.a_1 = 0.1
        self.a_2 = -0.0002
        
        # Gear ratio, effective radius, mass + inertia
        self.GR = 0.35
        self.r_e = 0.3
        self.J_e = 10
        self.m = 2000
        self.g = 9.81
        
        # Aerodynamic and friction coefficients
        self.c_a = 1.36
        self.c_r1 = 0.01
        
        # Tire force 
        self.c = 10000
        self.F_max = 10000
        
        # State variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0
        
        self.sample_time = 0.01
        
    def reset(self):
        # reset state variables
        self.x = 0
        self.v = 5
        self.a = 0
        self.w_e = 100
        self.w_e_dot = 0

class Vehicle(Vehicle):
    def step(self, throttle, alpha):
        # ==================================
        #  Implement vehicle model here
        Fg = self.m * self.g * np.sin(alpha)
        w_w = self.GR * self.w_e
        T_e = throttle * (self.a_0 + self.a_1 * self.w_e + self.a_2 * self.w_e * self.w_e)
        F_aero = self.c_a * self.v * self.v
        R_x = self.c_r1 * self.v
        F_load = Fg + F_aero + R_x
        s = (w_w * self.r_e - self.v) / (self.v)
        
        if abs(s) < 1:
            F_x = self.c * s
        else:
            F_x = self.F_max
        
        self.x = self.x + self.v * self.sample_time
        self.v = self.v + self.a * self.sample_time
        self.a = (F_x - F_load) / (self.m)
        self.w_e = self.w_e + self.w_e_dot * self.sample_time
        self.w_e_dot = (T_e - (self.GR * self.r_e * F_load)) / (self.J_e)           
                               
        # ==================================
        pass

sample_time = 0.01
time_end = 100
model = Vehicle()

t_data = np.arange(0,time_end,sample_time)
v_data = np.zeros_like(t_data)
# throttle percentage between 0 and 1
throttle = 0.5

# incline angle (in radians)
alpha = 0

for i in range(t_data.shape[0]):
    v_data[i] = model.v
    model.step(throttle, alpha)
    
plt.plot(t_data, v_data)
plt.grid(True)
plt.show()

time_end = 20
t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)
v_data = np.zeros_like(t_data)

# reset the states
model.reset()
# ==================================
#  Learner solution begins here
# ==================================
throttle_data = np.zeros_like(t_data)
throttle_data[0] = 0.2
for i in range(1, 500):
    throttle_data[i] = throttle_data[i-1] + 0.0006
for i in range(500, 1500):
    throttle_data[i] = throttle_data[i-1]
for i in range(1500, t_data.shape[0]):
    throttle_data[i] = throttle_data[i-1] - 0.001
    
alpha_data = np.zeros_like(t_data)
#alpha_data[0:500] = np.arctan(3/60)
#alpha_data[500:1500] = np.arctan(12/120)
#alpha_data[1500:2000] = 0
    
for i in range(t_data.shape[0]):
    x_data[i] = model.x
    v_data[i] = model.v
    
    if 0 <= x_data[i] <= 60:
        alpha_data[i] = np.arctan(3/60)
    elif 60 < x_data[i] <= 150:
        alpha_data[i] = np.arctan(12/120)
    else:
        alpha_data[i] = 0
            
    
    
    model.step(throttle_data[i], alpha_data[i])
    
# ==================================
#  Learner solution ends here
# ==================================

# Plot x vs t for visualization
print(str(throttle_data[1200]))
plt.plot(t_data, x_data)
plt.grid(True)
plt.show()
plt.plot(t_data, v_data)
plt.grid(True)
plt.show()
plt.plot(t_data, alpha_data)
plt.grid(True)
plt.show()
plt.plot(t_data, throttle_data)
plt.grid(True)
plt.show()

data = np.vstack([t_data, x_data]).T
np.savetxt('xdata.txt', data, delimiter=', ')

sample_time = 0.01
time_end = 30
model.reset()

t_data = np.arange(0,time_end,sample_time)
x_data = np.zeros_like(t_data)

# ==================================
#  Test various inputs here
# ==================================
for i in range(t_data.shape[0]):

    model.step(0,0)
    
plt.axis('equal')
plt.plot(x_data, y_data)
plt.show()
