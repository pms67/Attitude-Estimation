# Implementation of Complimentary Filter for MPU-6050 6-DOF IMU
# 
# Author: Philip Salmony [pms67@cam.ac.uk]
# Date: 4 August 2018

from imu import *
from time import sleep, time
from math import sin, cos, tan

imu = IMU()

# Iterations and sleep time
N = 1000
sleep_time = 0.01

# Filter coefficient
alpha = 0.1

print("Getting gyro bias...")
[bx, by, bz] = imu.get_gyro_bias(200)

# Complimentary filter estimates
phi_hat = 0.0
theta_hat = 0.0

# Measured sampling time
dt = 0.0
start_time = time()

for i in range(N):
    dt = time() - start_time
    start_time = time()
    
    # Get estimated angles from raw accelerometer data
    [phi_hat_acc, theta_hat_acc] = imu.get_acc_angles()
    
    # Get raw gyro data and subtract biases
    [p, q, r] = imu.get_gyro()
    p -= bx
    q -= by
    r -= bz
    
    # Calculate Euler angle derivatives 
    phi_dot = p + sin(phi_hat) * tan(theta_hat) * q + cos(phi_hat) * tan(theta_hat) * q
    theta_dot = cos(phi_hat) * q - sin(phi_hat) * r
    
    # Update complimentary filter
    phi_hat = (1 - alpha) * (phi_hat + dt * phi_dot) + alpha * phi_hat_acc
    theta_hat = (1 - alpha) * (theta_hat + dt * theta_dot) + alpha * theta_hat_acc   
    
    print("Phi: + " str(round(phi_hat, 1)) + " | Theta: " + str(round(theta_hat, 2)))
    
    sleep(sleep_time)
