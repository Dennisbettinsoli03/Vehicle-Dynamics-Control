
%% track 

clc
clear all
close all
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

%% Vehicle Parameters

l1 = 1.2;  % Front axle-c.o.g. distance (m)
l2 = 1.6;  % Rear axle-c.o.g. distance (m)
m = 1575;   % Vehicle mass (kg)
Jz = 4000;  % Yaw inertia (kg*m^2))
C1 = 27e3;  % Front tire stiffness
C2 = C1;    % Rear tire stiffness

%% PID design

speed = input('select initial vehicle speed in km/h \n');

% Linearization point
Xl=0;
Yl=0;
psil=psir(1);
vxl =speed/3.6;     % Speed assumed for controller design
vyl=0;
wpsil=0;
zel=[Xl;Yl;psil;vxl;vyl;wpsil];



%% Simulation

% % Initial conditions. 
% X0=0;
% Y0=0;
% psi0=0;
% vx0=20/3.6;     % Speed assumed for controller design
% vy0=0;
% wpsi0=0;
% ze0=[X0;Y0;psi0;vx0;vy0;wpsi0];

% Simulation time
Tfin=300;

%% Simulation 2: testing of different controllers

% loop over several controllers and 2 velocities
% after the tuning phase the PID parameters have been stored in the
% following arrays
% Define PID parameters for different controllers
Kp = [0.00316787757144982, 0.000205851292795625, 0.0074939, 0.00235493739727907];  % Proportional gains
Ki = [4.53622264721489e-05, 0,  0.00014903 ,0 ];  % Integral gains
Kd = [0.0491545117988438, 0.0188359493884402,  0.08373, 0.0816667774771917]; % Derivative gains
Fc = [280.508292919597,142.723194187966, 389.5583, 376.584482352379]; % First coefficient

% Define Stanley parameter
delta_m = 35 * pi / 180; % rad, maximum steering angle

file_bis='Project_2_2_bis_sim.slx';
open(file_bis)
for n = 3:5
    
    % Set PID parameters for the current controller
    if n<=4 
        sel = 2; % select PID controller
        Kp_current = Kp(n);
        Ki_current = Ki(n);
        Kd_current = Kd(n);
        Fc_current = Fc(n);
    elseif n ==5
        sel = 1; % select Stanley controller
       
    end
    % switch control: used to consider only the cross track error for the
    % cases 1 and 2
    if n==1 || n==2
        sw = 0;
    elseif n==3 || n==4 || n==5
        sw = 1;
    else 
        error('loop error')
    end

    % Run the simulation with the current PID parameters
    out = sim(file_bis);


    %% Plot pose (X vs Y) and time series of yaw rate, vx, vy

    % extract main sim data
    X = out.pose_PID.Data(1,:);
    Y = out.pose_PID.Data(2,:);
    ct_err = squeeze(out.cross_track_err.Data); % Cross track error
    h_err = squeeze(out.heading_err.Data); % heading error
    time = squeeze(out.tout); % Time


    if n == 1
        figure('Name', 'PID - cross track error', 'NumberTitle', 'off');
    elseif n == 2
        figure('Name', 'PD - cross track error', 'NumberTitle', 'off');
    elseif n == 3
        figure('Name', 'PID - cross track and heading error', 'NumberTitle', 'off');
    elseif n == 4
        figure('Name', 'PD - cross track and heading error', 'NumberTitle', 'off');
    elseif n == 5
        figure('Name', 'Stanley controller', 'NumberTitle', 'off');
    end

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
    ylabel('Heading Error (deg)')
    xlabel('Time (s)')
    title('Heading Error Over Time')
    grid on
    % Create a new figure for time series outputs from the simulation
    
    % Extract additional outputs from the simulation
    delta = squeeze(out.delta.Data);  % Steering angle
    omega_dot = squeeze(out.omegaphi_dot.Data);  % angular acceleration
    vy_dot = squeeze(out.vy_dot.Data);  % Lateral acceleration
    vx = squeeze(out.vx.Data);  % Longitudinal velocity
    

    if n == 1
        figure('Name', 'PID - ct vehicle signals', 'NumberTitle', 'off');
    elseif n == 2
        figure('Name', 'PD - ct vehicle signals', 'NumberTitle', 'off');
    elseif n == 3
        figure('Name', 'PID - ct&h vehicle signals', 'NumberTitle', 'off');
    elseif n == 4
        figure('Name', 'PD - ct&h vehicle signals', 'NumberTitle', 'off');
    elseif n == 5
        figure('Name', 'Stanley controller - vehicle signals', 'NumberTitle', 'off');
    end

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
end
