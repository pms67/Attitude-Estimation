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

% Optional: Filter accelerometer data
accFilter = false;

if (accFilter)
    windowSize = 20;
    b = (1/windowSize) * ones(1, windowSize);
    a = 1;
    phi_hat_acc = filter(b, a, phi_hat_acc);
    theta_hat_acc = filter(b, a, theta_hat_acc);
end

% Gyroscope only
phi_hat_gyr = zeros(1, length(t));
theta_hat_gyr = zeros(1, length(t));

% Optional: Filter gyroscope data
gyrFilter = false;

if (gyrFilter)
    windowSize = 20;
    b = (1/windowSize) * ones(1, windowSize);
    a = 1;
    phi_hat_gyr = filter(b, a, phi_hat_gyr);
    theta_hat_gyr = filter(b, a, theta_hat_gyr);
end

for i = 2:length(t)
   phi_hat_gyr(i) = phi_hat_gyr(i - 1) + Gx(i) * dt;
   theta_hat_gyr(i) = theta_hat_gyr(i - 1) + Gy(i) * dt;
end

% Complimentary Filter
alpha = 0.1;

phi_hat_complimentary = zeros(1, length(t));
theta_hat_complimentary = zeros(1, length(t));

for i=2:length(t)

    phi_hat_complimentary(i) = (1 - alpha) * (phi_hat_complimentary(i - 1) + Gx(i) * dt) + alpha * phi_hat_acc(i);
    theta_hat_complimentary(i) = (1 - alpha) * (theta_hat_complimentary(i - 1) + Gy(i) * dt) + alpha * theta_hat_acc(i);
    
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
   
    % Predict
    state_estimate = A * state_estimate + B * [Gx(i) Gy(i)]';
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