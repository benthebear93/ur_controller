import numpy as np
import rtde_control
import time

rtde_c = rtde_control.RTDEControlInterface("192.168.0.12")

# Task parameters
dt = 1.0/500 # 500Hz control loop
task_frame = [0, 0, 0, 0, 0, 0]
compliance_vector = [1, 1, 1, 0, 0, 0]

# Spiral movement parameters 
dTheta = 6*np.pi/10
k = 0.001
z_force = 0 # -10
vMax = [0.1*1.0, 0.1*1.0, 0.1*1.0, 0.1*0.5, 0.1*0.5, 0.1*0.5]

rtde_c.zeroFtSensor() # reset force sensor due to drift
time.sleep(0.2)
i = 0

try:
    while True:       
        start = time.time()             
        Fx = dTheta*k*(np.cos(dTheta*dt*i) - dTheta*i*np.sin(dTheta*dt*i))
        Fy = dTheta*k*(np.sin(dTheta*dt*i) + dTheta*i*np.cos(dTheta*dt*i))   
        Fz = z_force
       
        F_vec = [Fx, Fy, Fz, 0, 0, 0]
        print(F_vec)
        
        # control robot in force mode
        rtde_c.forceMode(task_frame, compliance_vector, F_vec, 2, vMax)
        end = time.time()
        duration = end - start
        
        if duration < dt:
            time.sleep(dt - duration)   
          
        i += 1
except KeyboardInterrupt:
    print("Stopped spiral search")
    rtde_c.forceModeStop()