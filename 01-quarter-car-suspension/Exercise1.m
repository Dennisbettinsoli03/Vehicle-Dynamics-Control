%% Exercise 1: Frequency response analysis

clc 
clear
close all

% Plot the Bode diagram of a quarter-car model in 3 different conditions:
% with a passive damper, with a skyhook and with a groundhook.
% The transfer functions plotted are:
% Sprung mass acceleration vs vertical displacement: zs_dotdot/ r
% Vertical load variable component vs vertical displacement force: Ft /P_r
% Actuator force vs vertical displacement force

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

% define frequency range from 0.1 to 100 Hz
f = linspace(10^-3, 100, 1000); 

figure(2)
hold on; grid on;

% Three possible working conditions
for n = 1:3
    figure(1)
    subplot(3, 1, n)
    
    for damp = 1:3
        switch damp
            case 1  % passive damper
                zeta0  = zeta; 
                zeta_m = 0;
                zeta_d = 0;
            case 2  % skyhook application
                zeta0  = 0.43;
                zeta_d = 0.25;
            case 3  % groundhook application
                zeta0  = 0.43;
                zeta_d = -0.25;
        end
        
        % define matrices according to the quarter car model
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
    
        % state space representation
        sys = ss(A, B, C, D);
        [mag, phase, wout] = bode(sys, f);
         

        % Plot the magnitude response of the 3 trasfer functions in every
        % working condition
        figure(1)
        hold on; 
        if n == 1
            plot(wout, squeeze(mag(n, :, :)) .* wout)
        else 
            plot(wout, squeeze(mag(n, :, :)) .* wout / P)
        end
        grid on;
        xlabel('Frequency (rad/s)');
        ylabel('Magnitude');
        legend('Passive damper', 'Skyhook', 'Groundhook', 'Location', 'best');
        if n==1 
            title('Sprung mass acceleration over vertical displacement');
        elseif n==2
            title('Vertical load variable component over vertical displacement force')
        elseif n==3
            title('Actuator force over vertical displacement force')
        end

        % Calculate eigenvalues for the current state-space system
        if n == 1
            figure(2)
            hold on
            eigenvalues = eig(A);
            % plot eigenvalues
            plot (real(eigenvalues),imag(eigenvalues),'x', 'MarkerSize', 10, 'LineWidth', 2)
        end
    if n == 1
        figure(2)
        grid on;
        xlabel('Real axis (\sigma)');
        ylabel('Imaginary axis (j\omega)');
        title('Eigenvalues (Pole Map) of the Quarter Car');
        legend('Passive damper', 'Skyhook', 'Groundhook', 'Location', 'best');
    end
    end
    hold off;

end


