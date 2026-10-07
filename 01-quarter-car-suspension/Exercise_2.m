%% Exercise 2.1 - Road Profile Excitation Driver Script

% Simulate a quarter-car model at different vehicle speeds using active skyhook and groundhook
% control.
clc
clear
close all

% define data
ms = 400; % kg
mu = 50;  % kg
K  = 25000;  % N/m
P  = 150000; % N/m
c  = 1500; % Ns/m
ct = 0;   % Ns/m

% define parameters zeta, the ratio between damping coefficient and the
% critical values
zeta   = c / (2 * sqrt(K * ms)); % Calculate the damping ratio
zeta_t = ct / (2 * sqrt(K * ms));

% define a vector of speeds
speeds = [10 30 50];

% for cycle over speeds
figure
for n = 1:length(speeds)
    V= speeds(n);
    subplot(length(speeds), 1, n)
    hold on
    title(['Actuator force over actuator velocity Vehicle speed = ', num2str(V), ' m/s'])
    ylabel('Actuator force (N)')
    xlabel('Actuator velocity (m/s)');

    for damp = 1:3
        switch damp 
            case 1  % passive damper
                zeta0  = zeta; 
                zeta_m = 0;
                zeta_d = 0;
            case 2  % skyhook 
                zeta0  = 0.43;
                zeta_d = 0.25;
            case 3  % groundhook
                zeta0  = 0.43;
                zeta_d = -0.25;
        end
        
        % define matrices
        A = [   0,                                        1,            0,                                              0,    0;
            -K/ms, -(2*(K*ms)^(1/2)*(zeta0 + zeta_d))/ms,         K/ms,           (2*(K*ms)^(1/2)*(zeta0 - zeta_d))/ms,    0;
                0,                                        0,            0,                                              1,    0;
             K/mu,  (2*(K*ms)^(1/2)*(zeta0 + zeta_d))/mu,  -(K + P)/mu, -(2*(K*ms)^(1/2)*(zeta0 - zeta_d + zeta_t))/mu, P/mu;
                0,                                        0,            0,                                              0,    0];
        
        B = [                           0;
                                        0;
                                        0;
               (2*zeta_t*(K*ms)^(1/2))/mu;
                                        1];
        
        C = [-K/ms, -(2*zeta0*(K*ms)^(1/2) + 2*zeta_d*(K*ms)^(1/2))/ms, K/ms, (2*zeta0*(K*ms)^(1/2) - 2*zeta_d*(K*ms)^(1/2))/ms, 0;
              0,                                                  0,   -P,                                                 0, P;
                0,                             -2*zeta_d*(K*ms)^(1/2),    0,                            -2*zeta_d*(K*ms)^(1/2), 0];
 
        D = [                        0;
                 2*zeta_t*(K*ms)^(1/2);
                                     0];
        % simulink simulation
        out = sim("Exercise2SIM.slx");

        % receive data from workspace
        V_act = out.V_act;
        F_act = out.F_act;

        % plot actuator force over velocity at given vehicle speed
        plot(V_act, F_act,'.', 'MarkerSize', 2)
        

    end
    legend('Passive damper', 'Skyhook', 'Groundhook', 'Location', 'best');

    hold off;
end

%% Exercise 2.2 Real World skyhook on a semi-active suspension

% Minimize sprung mass acceleration implementing a real world skyhook by
% tuning the damping parameter cs
V = 20;

% Maximum damping constraint
c_max = 4000;  % Ns/m

% matrices definitions
% State equation X: zs, z_dots, zu, z_dotu, r
A = [   0,  1,           0,  0,    0;
    -K/ms,  0,        K/ms,  0,    0;
        0,  0,           0,  1,    0;
     K/mu,  0, -(K + P)/mu,  0, P/mu;
        0,  0,           0,  0,    0];

B = [0,     0;
     0,  1/ms;
     0,     0;
     0, -1/mu;
     1,     0];

% y equation: acceleration ms, speed ms, speed mu
C = [-K/ms, 0, K/ms, 0, 0;  
         0, 1,    0, 0, 0;  
         0, 0,    0, 1, 0]; 

D = [0, 1/ms;  
     0,    0;
     0,    0];

% tuning parameter cs
cs_tune = [0,500,1000,2000,3000,4000];
rms_acc = zeros(1,length(cs_tune));

figure; hold on;
sgtitle('Acceleration of Sprung Mass with semi-active suspension');
fprintf('---------------REAL WORLD SKYHOOK IMPLEMENTATION - RESULTS---------------\n')
for i= 1:length(cs_tune)
    cs = cs_tune(i);
    % Simulink simulation
    out = sim('exercise2_2SIM.slx');
    
    % Collect the acceleration of the sprung mass as output of the sim
    acc_sprung = acc;
    
    % Plot the acceleration of the sprung mass
    subplot(3,2,i);
    plot(acc_sprung);
    xlabel('Time (s)');
    ylabel('Acceleration (m/s^2)');
    xlim([0 length(acc_sprung)]);
    grid on;
    if i ==1
        title('Passive suspension');
    elseif i > 1
        title(['Damping c =',num2str(cs), ' Ns/m'])
    end
    
    % RMS acceleration
    % Calculate the RMS acceleration of the sprung mass
    rms_acc(i) = sqrt(mean(acc_sprung.^2));

    % Display the RMS acceleration
    fprintf('\n RMS Acceleration of Sprung Mass with damping c: %.0f Ns/m is: %.2f m/s^2 \n', cs ,rms_acc(i) );
    if i > 1
        delta_rms = (rms_acc(i)-rms_acc(1)) / rms_acc(1) * 100;
        fprintf('RMS  Acceleration of Sprung Mass is reduced by: %.2f %%\n',abs(delta_rms))
    end
end

% Maximum and minimum accelerations
[acc_max, idx_max] = max(rms_acc);
[acc_min, idx_min] = min(rms_acc);
fprintf('\n------------MAXIMUM AND MINIMUM ACCELERATIONS----------------------------\n')
fprintf('The worst RMS acceleration is: %.2f m/s^2 \n obtained with a damping cs= %.0f Ns/m  \n', acc_max, cs_tune(idx_max));
fprintf('The best RMS acceleration is: %.2f m/s^2 \n  obtained with a damping cs= %.0f Ns/m \n', acc_min, cs_tune(idx_min));