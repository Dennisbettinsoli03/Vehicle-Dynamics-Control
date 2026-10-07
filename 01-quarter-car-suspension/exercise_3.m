%% Exercise 3: Quarter-car nonlinear dynamics and semi-active suspension
% 2 parts: 
% Part 1 -> keeping fixed the maximum current I_c, tune K_p to evaluate the
% response of the system related to RMS values and maximum values of
% acceleration and tire-ground force
% Part 2 -> repeat the same analysis keeping K_p fixed and tuning I_c

clear 
close all
clc

%% Part 1
% define data
ms = 400; % kg
mu = 50;  % kg
K  = 25000;  % N/m
P  = 150000; % N/m
c  = 1500; % Ns/m
ct = 0;   % Ns/m
I_c = 1.6; % upper current limit

fprintf('K_p optimization \n');
% select road type
inp = 1;
switch inp
    case 1  % random road profile
         'Random';
    case 2  % bump road profile
         'Bump';
    
end

% CASE 1: random road profile

K_p_tuning = linspace(0.2,0.6,20); % random signal, parameter to tune
acc_RMS = zeros(1,length(K_p_tuning)); % initialize RMS acceleration vector
force_RMS = zeros(1,length(K_p_tuning)); % initialize RMS tire ground force vector
acc_Max = zeros(1,length(K_p_tuning));  % initialize maximum acceleration vector
force_Max = zeros(1,length(K_p_tuning)); % initialize maximum force vector

% for cycle over the road profiles
for k = 1:2
    inp = k;

    % for cycle over tuning parameter
    for i = 1:length(K_p_tuning)
        K_p = K_p_tuning(i);
    
        % simulink sim
        sim("exercise_3sim.slx");
        
        acc_sprung = Acc_sprung;          % acceleration of the sprung mass
        tr_force = Tire_road_force;       % tire road force
        
        % calculate the RMS value of the acceleration and force
        acc_RMS (i) = sqrt(mean(acc_sprung.^2));
        force_RMS (i)  = sqrt(mean(tr_force.^2));
        acc_Max (i) = max(acc_sprung);
        force_Max (i) = max(tr_force);
    end
    
    % Plot to identify best values of K_p
    % Plot RMS acceleration and force for random road profile
    figure;
     if inp ==1
        sgtitle('Kp optimization for random road profile');
    elseif inp ==2 
         sgtitle('Kp optimization for bump road profile');  
    end
    subplot(2,2,1);
    plot(K_p_tuning, acc_RMS, '-o');
    xlabel('K_p Tuning Parameter');
    ylabel('RMS Acceleration (m/s^2)');
    grid on;
    title('RMS Acceleration vs K_p');

    subplot(2,2,2);
    plot(K_p_tuning, force_RMS, '-o');
    xlabel('K_p Tuning Parameter');
    ylabel('RMS Tire Road Force (N)');
     if inp ==1
        title('RMS Tire Road Force vs K_p for Random Road Profile');
    elseif inp ==2
        title('RMS Tire Road Force vs K_p for Bump Road Profile')
    else 
        error('Warning: input parameter non valid')
    end
    grid on;
    
    subplot(2,2,3);
    plot(K_p_tuning, acc_Max, '-o');
    xlabel('K_p Tuning Parameter');
    ylabel('Maximum Acceleration (m/s^2)');
    title('Maximum Acceleration vs K_p');
    grid on;
    
    subplot(2,2,4);
    plot(K_p_tuning, force_Max, '-o');
    xlabel('K_p Tuning Parameter');
    ylabel('Maximum Tire Road Force (N)');
    title('Maximum Tire Road Force vs K_p');

    grid on;
    
    % Find optimal parameters based on minimum RMS and maximum values
    [opt_RMS_acc, idx_RMS_acc] = min(acc_RMS);
    opt_kp_RMS_acc = K_p_tuning(idx_RMS_acc);

    [opt_RMS_force, idx_RMS_force] = min(force_RMS);
    opt_kp_RMS_force = K_p_tuning(idx_RMS_force);
    
    [opt_Max_acc, idx_Max_acc] = min(acc_Max);
    opt_kp_Max_acc = K_p_tuning(idx_Max_acc);

    [opt_Max_force, idx__Max_force] = min(force_Max);
    opt_kp_Max_force = K_p_tuning(idx__Max_force);

    % Display results for the current input
    if inp == 1
        fprintf('Results for random input \n');
    elseif inp ==2
        fprintf('Results for bump input \n');
    end
    fprintf('Optimal RMS Acceleration = %.2f, reached with K_p = %.2f\n', opt_RMS_acc, opt_kp_RMS_acc);
    fprintf('Optimal RMS Tire Road Force = %.2f, reached with K_p = %.2f\n',opt_RMS_force, opt_kp_RMS_force);
    fprintf('Otimal Maximum Acceleration = %.2f, reached with K_p = %.2f\n',opt_Max_acc, opt_kp_Max_acc);
    fprintf('Optimal Maximum Tire Road Force = %.2f, reached with K_p = %.2f\n',opt_Max_force, opt_kp_Max_force);

end

% second part: fixed K_p and variable current I
I_c_tuning = linspace(0.29,1.6,20);
K_p = 0.3;

fprintf('I_c optimization \n');
for k = 1:2
    inp = k;
    for i = 1:length(I_c_tuning)
        I_c = I_c_tuning(i);
    
        % simulink sim
        sim("exercise_3sim.slx");
        
        acc_sprung = Acc_sprung;          % acceleration of the sprung mass
        tr_force = Tire_road_force;       % tide road force
        
        % calculate the RMS value of the acceleration and force
        acc_RMS (i) = sqrt(mean(acc_sprung.^2));
        force_RMS (i)  = sqrt(mean(tr_force.^2));
        acc_Max (i) = max(acc_sprung);
        force_Max (i) = max(tr_force);
    end
    
    % Plot to identify best values of I_c
    % Plot RMS acceleration and force for random road profile
    figure;
    if inp ==1
        sgtitle('Ic optimization for random road profile');
    elseif inp ==2 
         sgtitle('Ic optimization for bump road profile');  
    end
    subplot(2,2,1);
    plot(I_c_tuning, acc_RMS, '-o');
    xlabel('I_c Tuning Parameter');
    ylabel('RMS Acceleration (m/s^2)');
    xlim([I_c_tuning(1) I_c_tuning(end)]);
    grid on;
    title('RMS Acceleration vs I_c');

    subplot(2,2,2);
    plot(I_c_tuning, force_RMS, '-o');
    xlabel('I_c Tuning Parameter');
    ylabel('RMS Tire Road Force (N)');
    xlim([I_c_tuning(1) I_c_tuning(end)]);
    title('RMS Tire Road Force vs I_c');
    grid on;

    subplot(2,2,3);
    plot(I_c_tuning, acc_Max, '-o');
    xlabel('I_c Tuning Parameter');
    ylabel('Maximum Acceleration (m/s^2)');
    xlim([I_c_tuning(1) I_c_tuning(end)]);
    title('Maximum Acceleration vs I_c');
    grid on;
    
    subplot(2,2,4);
    plot(I_c_tuning, force_Max, '-o');
    xlabel('I_c Tuning Parameter');
    ylabel('Maximum Tire Road Force (N)');
    xlim([I_c_tuning(1) I_c_tuning(end)]);
    title('Maximum Tire Road Force vs I_c');
    grid on;
    
    % Find optimal parameters based on minimum RMS and maximum values
    [opt_RMS_acc, idx_RMS_acc] = min(acc_RMS);
    opt_Ic_RMS_acc = I_c_tuning(idx_RMS_acc);
    [opt_RMS_force, idx_RMS_force] = min(force_RMS);
    opt_Ic_RMS_force = I_c_tuning(idx_RMS_force);
    
    [opt_Max_acc, idx_Max_acc] = min(acc_Max);
    opt_Ic_Max_acc = I_c_tuning(idx_Max_acc);
    [opt_Max_force, idx__Max_force] = min(force_Max);
    opt_Ic_Max_force = I_c_tuning(idx__Max_force);
    
    % Display results for the current input
    if inp == 1
        fprintf('Results for random input \n');
    elseif inp == 2
        fprintf('Results for bump input \n');
    end
    fprintf('Optimal RMS Acceleration = %.2f, reached with I_c = %.2f\n', opt_RMS_acc, opt_Ic_RMS_acc);
    fprintf('Optimal RMS Tire Road Force = %.2f, reached with I_c = %.2f\n',opt_RMS_force, opt_Ic_RMS_force);
    fprintf('Optimal Maximum Acceleration = %.2f, reached with I_c = %.2f\n',opt_Max_acc, opt_Ic_Max_acc);
    fprintf('Optimal Maximum Tire Road Force = %.2f, reached with I_c = %.2f\n',opt_Max_force, opt_Ic_Max_force);
end

%% Part 2: bode diagram
% linearization of the system
% define a constant value of the current and the parameter K_p
load linearization.mat
I_c = 0.3;
K_p = 0;