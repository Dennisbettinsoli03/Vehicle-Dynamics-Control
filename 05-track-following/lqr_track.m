%% INITIALIZATION
clear all;
close all;
clc;

%% REFERENCE TRAJECTORY GENERATION
%% Data Loading
gps_data = readmatrix('Track_data.xlsx');
lat = gps_data(:, 1);
lon = gps_data(:, 2);

%% Lat/Lon to X/Y Conversion (Flat Earth Approximation)
% Set the first GPS point as the origin (0,0) of our local frame
lat0 = lat(1);
lon0 = lon(1);

R_earth = 6371000; % Earth radius in meters

% Convert degrees to radians for trigonometric formulas
lat_rad = deg2rad(lat);
lon_rad = deg2rad(lon);
lat0_rad = deg2rad(lat0);
lon0_rad = deg2rad(lon0);

% Compute X and Y coordinates in meters relative to the starting point
Xr = R_earth .* (lon_rad - lon0_rad) .* cos(lat0_rad);
Yr = R_earth .* (lat_rad - lat0_rad);

%% Data Smoothing
% use a moving average to smooth data
filter_window = 10; % Adjust this value: higher means a smoother curve
Xr = smoothdata(Xr, 'movmean', filter_window);
Yr = smoothdata(Yr, 'movmean', filter_window);

%% Heading Reference Calculation (psi_ref)
% Compute the tangent angle to the path
psir = [atan2(diff(Yr), diff(Xr)); 0];
psir(end) = psir(end-1); 

%% Final Matrix Creation
refPoses = [Xr, Yr, psir];

% Sanity check plot to verify the result
figure('Name', 'Converted Real Circuit');
plot(Xr, Yr, 'r--', 'LineWidth', 1.5);
axis equal; % Crucial to visualize correct physical proportions!
grid on;
title('Reference Trajectory in Meters');
xlabel('X [m]'); 
ylabel('Y [m]');

speed = input('select vehicle speed\n');
%% VEHICLE & DYNAMIC SINGLE-TRACK ERROR (DSTE) MODEL PARAMETERS

% Vehicle parameters
a  = 1.2;       % [m] Distance from CoG to front axle
b  = 1.6;       % [m] Distance from CoG to rear axle
m  = 1575;      % [kg] Vehicle mass
Iz = 4000;      % [kg*m^2] Yaw moment of inertia
C1 = 27e3;      % [N/rad] Front cornering stiffness
C2 = 20e3;      % [N/rad] Rear cornering stiffness
vx = speed / 3.6;  % [m/s] Longitudinal velocity (constant)

% State-Space Matrices for the DSTE Model

% State vector x = [e_ct; e_ct_dot; psi_e; psi_e_dot]
A = [0, 1, 0, 0;
     0, -(C1+C2)/(m*vx), (C1+C2)/m, (b*C2 - a*C1)/(m*vx);
     0, 0, 0, 1;
     0, (b*C2 - a*C1)/(Iz*vx), (a*C1 - b*C2)/Iz, -(a^2*C1 + b^2*C2)/(Iz*vx)];

B = [0;    
     C1/m;
     0;
     a*C1/Iz];

% Disturbance matrix (road curvature feed-forward term)
E = [0;
    (b*C2 - a*C1)/(m*vx) - vx;
    0;
    -(a^2*C1 + b^2*C2)/(Iz*vx)];

% Check open-loop stability
eigen_open = eig(A);
fprintf('--- OPEN-LOOP ANALYSIS ---\n');
fprintf('Open-loop eigenvalues: \n');
disp(eigen_open);

%% LQR CONTROLLER DESIGN (BRYSON'S RULE)

% Define maximum acceptable limits for the states and control input
e1_max    = 0.5;       % [m] Maximum cross-track error
e2_max    = 4 * pi/180; % [rad] Maximum heading error
delta_max = 4 * pi/180; % [rad] Maximum steering angle (comfort limit)

% Weighting matrices Q (states) and R (input)
Q = diag([1/e1_max^2, 0, 1/e2_max^2, 0]);
R = 1/delta_max^2;

% Compute optimal LQR gain matrix K
K = lqr(A, B, Q, R);

% Extract equivalent PD controller gains from the LQR matrix K
P1 = K(1); % Proportional gain for lateral error
D1 = K(2); % Derivative gain for lateral error
P2 = K(3); % Proportional gain for heading error
D2 = K(4); % Derivative gain for heading error

% Compute derivative filter coefficients (N) to mitigate high-frequency noise
tau_d1 = D1 / P1;
N1 = 10 / tau_d1; % Filter pole should be 10x faster than controller dynamics

tau_d2 = D2 / P2;
N2 = 10 / tau_d2;

% Check closed-loop stability
F = A - B*K;
eigen_closed = eig(F);
fprintf('\n--- CLOSED-LOOP ANALYSIS ---\n');
fprintf('Closed-loop eigenvalues:\n');
disp(eigen_closed);

%% 4. SIMULATION SETUP & EXECUTION
% Initial conditions for the DSTP non-linear model
X0 = 0;
Y0 = 0;
psi0 = refPoses(1,3);
vx0 = speed / 3.6; % m/s
vy0 = 0;
wpsi0 = 0;
ze0 = [X0; Y0; psi0; vx0; vy0; wpsi0];

% Simulation settings
Tfin = 150; % [s] Total simulation time
Ts = 0.01;  % [s] Data sampling time

% Run Simulink model
open('lqr_sim.slx'); % Ensure the model is open
out = sim('lqr_sim.slx');
fprintf('Simulation completed.\n');

%% DATA EXTRACTION & POST-PROCESSING
% Extract data 
time = out.tout;
e_ct       = squeeze(out.e_ct.Data);       % [m] Cross-track error
e_ct_dot   = squeeze(out.e_ct_dot.Data);   % [m/s] Cross-track error derivative
psi_e      = squeeze(out.psi_e.Data);      % [rad] Heading error
psi_e_dot  = squeeze(out.psi_e_dot.Data);  % [rad/s] Heading error derivative
delta_tun      = squeeze(out.delta_tun.Data);      % [rad] Steering angle
delta_filt = squeeze(out.delta_filtered.Data); % delta after actuator
%% PERFORMANCE METRICS EVALUATION
fprintf('\n--- LQR PERFORMANCE METRICS ---\n');

% Overshoot (Maximum peak error)
peak_ect = max(abs(e_ct));
overshoot = peak_ect / abs(e_ct(end)) *100 -100;
fprintf('Overshoot (Max e_ct): %.3f %% \n', overshoot);

% Settling Time (Time to remain within a 5cm tolerance)
settling_threshold = 0.05* abs(e_ct(end)); % [m]
idx_settling = find(abs(e_ct) > (abs(e_ct(end)) + settling_threshold) | (abs(e_ct) < (abs(e_ct(end)) - settling_threshold )),1,"last"); 
if isempty(idx_settling)
    t_settling = 0;
else
    t_settling = time(idx_settling);
end
fprintf('Settling Time (5cm tolerance): %.2f s\n', t_settling);

% Steering Smoothness (Max Slew Rate in deg/s)
delta_deg = rad2deg(delta_tun);
steering_speed = diff(delta_deg) ./ diff(time);
max_slew_rate = max(abs(steering_speed));
fprintf('Steering Smoothness (Max Slew Rate): %.1f deg/s\n', max_slew_rate);

%% PLOTTING RESULTS
% Plot Lateral Error Dynamics
figure('Name', 'Lateral Error', 'NumberTitle', 'off');
plot(time, e_ct, 'b', 'LineWidth', 1.5); hold on;
plot(time, e_ct_dot, 'r--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Lateral Error');
legend('e_{ct} [m]', '\dot{e}_{ct} [m/s]');
title('Cross-Track Error Dynamics');
grid on;

% Plot Heading Error Dynamics
figure('Name', 'Heading Error', 'NumberTitle', 'off');
plot(time, psi_e, 'g', 'LineWidth', 1.5); hold on;
plot(time, psi_e_dot, 'm--', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Heading Error');
legend('\psi_e [rad]', '\dot{\psi}_e [rad/s]');
title('Orientation Error Dynamics');
grid on;

% Plot Steering Action
figure('Name', 'Steering Command', 'NumberTitle', 'off');
plot(time, delta_filt, 'k', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Steering Angle \delta [deg]');
title('Controller Output (Steering Effort)');
grid on;


%% DSTP MODEL SIMULATION

  %% Plot pose (X vs Y) and time series of yaw rate, vx, vy

% extract main sim data
X = out.pose_PID.Data(1,:);
Y = out.pose_PID.Data(2,:);
ct_err = squeeze(out.cross_track_err.Data); % Cross track error
h_err = squeeze(out.heading_err.Data); % heading error
time = squeeze(out.tout); % Time


figure('Name', 'LQR model - trajectory', 'NumberTitle', 'off');

hold on
plot(X,Y,'b','LineWidth',1.5);
X_ref= out.ref_trajectory.Data(:,1);
Y_ref = out.ref_trajectory.Data(:,2);
hold on
plot(Xr,Yr,'LineStyle','--','LineWidth',1.5);
xlabel('X (m)')
ylabel('Y (m)')
title('Vehicle trajectory')
legend('Controller trajectory','Reference trajectory')
grid on

figure
subplot(2,1,1)
plot(time, ct_err, 'r', 'LineWidth', 1.5);
ylabel('Cross Track Error (m)')
title('Cross Track Error Over Time')
grid on

subplot(2,1,2)
h_err_deg = h_err * 180 / pi;
plot(time, h_err_deg, 'g', 'LineWidth', 1.5);
hold on

% Create a new figure for time series outputs from the simulation

% Extract additional outputs from the simulation
delta = squeeze(out.delta.Data);  % Steering angle
omega_dot = squeeze(out.omegaphi_dot.Data);  % angular acceleration
vy_dot = squeeze(out.vy_dot.Data);  % Lateral acceleration
vx = squeeze(out.vx.Data);  % Longitudinal velocity


figure('Name', 'LQR model - vehicle signals', 'NumberTitle', 'off');

% Plot subplots
subplot(2,2,1)
plot(time,delta,'b','LineWidth',2)
ylabel('\delta (rad)')
title('Steering angle \delta')
xlim([0 time(end)])
grid on

subplot(2,2,2)
plot(time,omega_dot,'k','LineWidth',2)
ylabel('d\omega_{\phi}/dt (rad/s^2)')
xlabel('Time (s)')
title('Yaw acceleration \omega_{\phi} dot')
xlim([0 time(end)])
grid on

subplot(2,2,3)
plot(time,vy_dot,'m','LineWidth',2)
ylabel('a_y (m/s^2)')
title('Lateral acceleration / v_y dot')
xlim([0 time(end)])
grid on

subplot(2,2,4)
plot(time,vx,'b','LineWidth',2)
ylabel('v_x (m/s)')
xlabel('Time (s)')
title('Longitudinal velocity v_x')
xlim([0 time(end)])
grid on