% Sensor Fusion Demonstration
%
% Author: Philip Salmony
% Date: 2 August 2018
% Email: philip.salmony@gmail.com

% Parameters
dt = 0.0185;
%t = (0:(length(Ax) - 1))' * dt;

% Accelerometer only
phi_hat_acc = atan2(Ay, sqrt(Ax .^ 2 + Az .^ 2)) * 180.0 / pi;
theta_hat_acc = atan2(-Ax, sqrt(Ay .^ 2 + Az .^ 2)) * 180.0 / pi;

% Gyroscope only
phi_hat_gyr = zeros(1, length(t));
theta_hat_gyr = zeros(1, length(t));

for i = 2:length(t)
   p = Gx(i) * pi / 180.0;
   q = Gy(i) * pi / 180.0;
   r = Gz(i) * pi / 180.0;
   
   phi_hat = phi_hat_gyr(i - 1);
   theta_hat = theta_hat_gyr(i - 1);
    
   phi_hat_gyr(i) = phi_hat_gyr(i - 1) + dt * (p + sin(phi_hat) * tan(theta_hat) * q + cos(phi_hat) * tan(theta_hat) * r);
   theta_hat_gyr(i) = theta_hat_gyr(i - 1) + dt * (cos(phi_hat) * q - sin(phi_hat) * r);
   
   %phi_hat_gyr(i) = phi_hat_gyr(i - 1) + Gx(i) * dt;
   %theta_hat_gyr(i) = theta_hat_gyr(i - 1) + Gy(i) * dt;
end

phi_hat_gyr = phi_hat_gyr * 180.0 / pi;
theta_hat_gyr = theta_hat_gyr * 180.0 / pi;

% Complimentary Filter
alpha = 0.5;

phi_hat_complimentary = zeros(1, length(t));
theta_hat_complimentary = zeros(1, length(t));

for i=2:length(t)

    p = Gx(i) * pi / 180.0;
    q = Gy(i) * pi / 180.0;
    r = Gz(i) * pi / 180.0;
   
    phi_hat = phi_hat_complimentary(i - 1);
    theta_hat = theta_hat_complimentary(i - 1);
    
    phi_hat_gyr_comp = phi_hat_complimentary(i - 1) + dt * (p + sin(phi_hat) * tan(theta_hat) * q + cos(phi_hat) * tan(theta_hat));
    theta_hat_gyr_comp = theta_hat_complimentary(i - 1) + dt * (cos(phi_hat) * q - sin(phi_hat) * r);
       
    phi_hat_complimentary(i) = (1 - alpha) * (phi_hat_gyr_comp) + alpha * phi_hat_acc(i);
    theta_hat_complimentary(i) = (1 - alpha) * (theta_hat_gyr_comp) + alpha * theta_hat_acc(i);
    
end

% Kalman (https://www.mouser.co.uk/applications/sensor_solutions_mems/)
A = [1 -dt 0 0; 0 1 0 0; 0 0 1 -dt; 0 0 0 1];
B = [dt 0 0 0; 0 0 dt 0]';
C = [1 0 0 0; 0 0 1 0];
P = eye(4);
Q = eye(4) * 0.01;
R = eye(2) * 10;
state_estimate = [0 0 0 0]';

kal_phi_hat = zeros(1, length(t));
kal_bias_phi = zeros(1, length(t));

kal_theta_hat = zeros(1, length(t));
kal_bias_theta = zeros(1, length(t));

for i=2:length(t)
    
    p = Gx(i) * pi / 180.0;
    q = Gy(i) * pi / 180.0;
    r = Gz(i) * pi / 180.0;
   
    phi_hat = kal_phi_hat(i - 1);
    theta_hat = kal_theta_hat(i - 1);
    
    phi_dot = p + sin(phi_hat) * tan(theta_hat) * q + cos(phi_hat) * tan(theta_hat) * r;
    theta_dot = cos(phi_hat) * q - sin(phi_hat) * r;
       
   
    % Predict
    state_estimate = A * state_estimate + B * [phi_dot theta_dot]';
    P = A * P * A' + Q;
    
    % Update
    measurement = [phi_hat_acc(i) theta_hat_acc(i)]';
    y_tilde = measurement - C * state_estimate;
    S = R + C * P * C';
    K = P * C' * (S^-1);
    state_estimate = state_estimate + K * y_tilde;
    P = (eye(4) - K * C) * P;
    
    kal_phi_hat(i) = state_estimate(1);
    kal_bias_phi(i) = state_estimate(2);
    kal_theta_hat(i) = state_estimate(3);
    kal_bias_theta(i) = state_estimate(4);
    
end

% Plots
subplot(2, 1, 1);
plot(t, phi_hat_complimentary, t, phi_hat_acc, t, phi_hat_gyr, t, kal_phi_hat)
legend('Complimentary', 'Accelerometer', 'Gyro', 'Kalman')
xlabel('Time (s)')
ylabel('Angle (degrees)')
title('Roll')

subplot(2, 1, 2);
plot(t, theta_hat_complimentary, t, theta_hat_acc, t, theta_hat_gyr, t, kal_theta_hat)
legend('Complementary', 'Accelerometer', 'Gyro', 'Kalman')
xlabel('Time (s)')
ylabel('Angle (degrees)')
title('Pitch')
