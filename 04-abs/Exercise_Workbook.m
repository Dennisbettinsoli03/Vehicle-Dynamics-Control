%% *01USHLO - Driver Assistance System Design* 
% *Project 4 - Anti-lock Braking System design*

set(0,'defaultAxesFontName', 'Times New Roman',...
'defaultTextFontName', 'Times New Roman',...
'defaultAxesFontSize', 12,...
'defaultTextFontSize', 12,...
'defaultLineLineWidth',2,...
'DefaultAxesXGrid','on',...
'DefaultAxesYGrid','on',...
'DefaultTextInterpreter','Tex')
% Exercise 1: Driveline Model – Braking

clc
close all
clear

VehicleParameters;
load('LUT_values.mat');

m0  = m;            % baseline vehicle mass for the sensitivity study
Cx0 = Cx;           % baseline drag coefficient for the sensitivity study

Ts = 1e-3;

n_vec = n_compact;
T_vec = T_compact;
sigma_vec = sigma;
mu_vec = mu_dry;

V0 = 80/3.6;
omega0 = V0/Re;

Jdf = Jwf+Jt/tau_f^2;
Jdr = Jwr;
hc = 1; 

%% braking maneuver - Speed and slip trends
% % 1) Analyze the effect of $h_{b\;}$and $K_{b\;}$on stopping distance.

Traction = 2;                   % =1 (front wheel drive), =2 (rear wheel drive)
ha = 0;
t_pedalAct = 0.2;
t_shifting = 0.6;
n_shiftup = 450;                % rad/s
n_shiftdown = 140;              % rad/s
startingGear = 5;
gear_vec = i_g;
Kb = 2.33;                      % Kb: front/rear brake ratio 
TbTot   = 3000;                 % total brake force
Tbf_max = TbTot/(1 + 1/Kb);     % Max brake torque front
Tbr_max = Tbf_max/Kb;           % Max brake torque rear

% braking command:
hb = 0.25;

% simulation:
out = sim("Exercise1.slx");

figure('Name','h_b_K_b_stoppingDistance_1','NumberTitle','off')
subplot(2,1,1)
title(sprintf('h_b = %g - K_b = %g',hb,Kb))
yyaxis left
plot(out.Speed.time, out.Speed.signals.values*3.6)
xlabel('Time [s]')
ylabel('Speed [km/h]')
yyaxis right
plot(out.Distance.time, out.Distance.signals.values)
ylabel('Distance [m]')

subplot(2,1,2)
title('Tire Slip Trends')
hold;
plot(out.slip.time, out.slip.signals(1).values)
plot(out.slip.time, out.slip.signals(2).values)
plot(out.slip.time, out.slip.signals(3).values)
plot(out.slip.time, out.slip.signals(4).values)
legend('Front Left tyre', 'Front Right tyre', 'Rear Left tyre', 'Rear Right tyre')
xlabel('Time [s]')
ylabel('Slip [-]')
ylim([-1 1])
% return

hb = 0.90;                       % brake command [0 : 1]
Kb = 1.7;                        % Kb: front/rear brake ratio [0.7 : 4] 
TbTot   = 3000;                  % total brake force
Tbf_max = TbTot/(1 + 1/Kb); 
Tbr_max = Tbf_max/Kb; 

out = sim("Exercise1.slx");

figure('Name','h_b_K_b_stoppingDistance_2','NumberTitle','off')
subplot(2,1,1)
title(sprintf('h_b = %g - K_b = %g',hb,Kb))
yyaxis left
plot(out.Speed.time, out.Speed.signals.values*3.6)
xlabel('Time [s]')
ylabel('Speed [km/h]')
yyaxis right
plot(out.Distance.time, out.Distance.signals.values)
ylabel('Distance [m]')

subplot(2,1,2)
title('Tire Slip Trends')
hold;
plot(out.slip.time, out.slip.signals(1).values)
plot(out.slip.time, out.slip.signals(2).values)
plot(out.slip.time, out.slip.signals(3).values)
plot(out.slip.time, out.slip.signals(4).values)
legend('Front Left tyre', 'Front Right tyre', 'Rear Left tyre', 'Rear Right tyre')
xlabel('Time [s]')
ylabel('Slip [-]')
ylim([-1 1])
% return

% % 2) Analyze stopping distance sensitivity to vehicle mass and drag coefficient in a ±15% range.
mass_coeff = 1.15;          % [0.85 : 1.15]
m = m0*mass_coeff;
Cx_coeff = 1.15;            % [0.85 : 1.15]
Cx = Cx0*Cx_coeff;

% braking condition:
hb = 0.45;
Kb = 2.33;                   % Kb: brake distribution factor. Usually from 50/50 to 70/30 (=2.33)
TbTot   = 3000;             % total brake force
Tbf_max = TbTot/(1 + 1/Kb); 
Tbr_max = Tbf_max/Kb; 

out = sim("Exercise1.slx");

figure('Name','vehicleMass_Cx_1','NumberTitle','off')
subplot(2,1,1)
title(sprintf('h_b = %g - K_b = %g / mass_{Coeff} = %g - Cx_{Coeff} = %g',hb,Kb,mass_coeff,Cx_coeff))
yyaxis left
plot(out.Speed.time, out.Speed.signals.values*3.6)
xlabel('Time [s]')
ylabel('Speed [km/h]')
yyaxis right
plot(out.Distance.time, out.Distance.signals.values)
ylabel('Distance [m]')

subplot(2,1,2)
title(sprintf('Tire Slip Trends / m = %g [kg] - Cx = %g',m,Cx))
hold on;
plot(out.slip.time, out.slip.signals(1).values)
plot(out.slip.time, out.slip.signals(2).values)
plot(out.slip.time, out.slip.signals(3).values)
plot(out.slip.time, out.slip.signals(4).values)
legend('Front Left tyre', 'Front Right tyre', 'Rear Left tyre', 'Rear Right tyre')
xlabel('Time [s]')
ylabel('Slip [-]')
ylim([-1 1])

mass_coeff = 0.85;          % [0.85 : 1.15]
m = m0*mass_coeff;
Cx_coeff = 0.85;            % [0.85 : 1.15]
Cx = Cx0*Cx_coeff;

% braking condition:
hb = 0.45;
Kb = 2.33;                   % Kb: brake distribution factor. Usually from 50/50 to 70/30
TbTot   = 3000;             % total brake force
Tbf_max = TbTot/(1 + 1/Kb); 
Tbr_max = Tbf_max/Kb; 

out = sim("Exercise1.slx");

figure('Name','vehicleMass_Cx_2','NumberTitle','off')
subplot(2,1,1)
title(sprintf('h_b = %g - K_b = %g / mass_{Coeff} = %g - Cx_{Coeff} = %g',hb,Kb,mass_coeff,Cx_coeff))
yyaxis left
plot(out.Speed.time, out.Speed.signals.values*3.6)
xlabel('Time [s]')
ylabel('Speed [km/h]')
yyaxis right
plot(out.Distance.time, out.Distance.signals.values)
ylabel('Distance [m]')

subplot(2,1,2)
title(sprintf('Tire Slip Trends / m = %g [kg] - Cx = %g',m,Cx))
hold on;
plot(out.slip.time, out.slip.signals(1).values)
plot(out.slip.time, out.slip.signals(2).values)
plot(out.slip.time, out.slip.signals(3).values)
plot(out.slip.time, out.slip.signals(4).values)
legend('Front Left tyre', 'Front Right tyre', 'Rear Left tyre', 'Rear Right tyre')
xlabel('Time [s]')
ylabel('Slip [-]')
ylim([-1 1])
% return

% % % 3) Compare the results obtained in the previous points to the stopping distance obtained with Neutral gear ($h_{c\;}$= 0).
Traction = 2;
ha = 0;
hc = 0;                 % Neutral! See hc and switch inside "engine + clutch" block
t_pedalAct   = 0.2;
t_shifting   = 0.6;
n_shiftup    = 450;
n_shiftdown  = 140;
startingGear = 5;
gear_vec     = i_g;
Kb = 2.33;               % 70/30
Tbr_max = Tbf_max/Kb;
hb = 1;                 % full brake pedal

m  = m0;                % restore nominal vehicle mass after the sensitivity study
Cx = Cx0;               % restore nominal drag coefficient

hc = 1;                 % geared: engine braking transmitted through the driveline
out = sim("Exercise1.slx");

hc = 0;                 % Neutral: clutch disengaged, no engine braking
neutral = sim("Exercise1.slx");

figure('Name','neutral_gear','NumberTitle','off')
subplot(2,1,1)
title(sprintf('h_b = %g - K_b = %g',hb,Kb))
hold on;
yyaxis left
plot(out.Speed.time, out.Speed.signals.values*3.6)
plot(neutral.Speed.time, neutral.Speed.signals.values*3.6)
xlabel('Time [s]')
ylabel('Speed [km/h]')
yyaxis right
plot(out.Distance.time, out.Distance.signals.values)
plot(neutral.Distance.time, neutral.Distance.signals.values)
ylabel('Distance [m]')

% subplot(3,1,2)
% title('Tire Slip Trends')
% hold on;
% plot(out.slip.time, out.slip.signals(1).values, 'SeriesIndex',1, 'DisplayName', 'FL tyre')
% plot(out.slip.time, out.slip.signals(2).values, 'SeriesIndex',2, 'DisplayName', 'FR tyre')
% plot(out.slip.time, out.slip.signals(3).values, 'SeriesIndex',2, 'DisplayName', 'RL tyre')
% plot(out.slip.time, out.slip.signals(4).values, 'SeriesIndex',4, 'DisplayName', 'RR tyre')
% xlabel('Time [s]')
% ylabel('Slip [-]')
% ylim([-1 1])
% legend()

subplot(2,1,2)  
title('Tire Slip Trends')
plot(neutral.slip.time, neutral.slip.signals(1).values, 'r-', 'SeriesIndex',1, 'DisplayName', 'FL tyre - neutral')
hold on
plot(neutral.slip.time, neutral.slip.signals(2).values, 'r-', 'SeriesIndex',2, 'DisplayName', 'FR tyre - neutral')
plot(neutral.slip.time, neutral.slip.signals(3).values, 'g-', 'SeriesIndex',2,'DisplayName', 'RL tyre - neutral')
plot(neutral.slip.time, neutral.slip.signals(4).values, 'g-', 'SeriesIndex',4, 'DisplayName', 'RR tyre - neutral')
xlabel('Time [s]')
ylabel('Slip [-]')
ylim([-1 1])
legend()
% return

%% Exercise 2: ABS Controller for Wheel Slip Regulation
% 1) Complete the Simulink scheme of «Exercise1.slx» with the following controller 
% on each wheel and run the simulation with $K_{\mathrm{gain}}$ = -600.

% % $$\left\lbrace \begin{array}{ll}\dot{u} =K_{\mathrm{gain}} \mathrm{sign}\left(\sigma_r 
% -\sigma \;\right) & \\T_{\mathrm{brake}} =\mathrm{sat}\left(u,0,T_{b\left(f,r\right)} 
% \right) & \end{array}\right.$$

% 2) Explain why the value of $\sigma_r$ was selected as -0.2 and what would 
% % happen if it was set to -0.1 and -0.3 respectively. Compare the results of the 
% % simulations, especially the slip trend.

Traction = 2;
ha = 0;
hc = 0;                 % Neutral! See hc and switch inside "engine + clutch" block
Kb = 2.33;              % 70/30
Tbr_max = Tbf_max/Kb;
hb = 0.35;

sigma_r_vec = [-0.3 -0.2 -0.1];
K_gain  = -600;
wheelB_controller = 2;      % =2 set the controller
Tfin = 300;

for i = 1:length(sigma_r_vec)
    sigma_r = sigma_r_vec(i);
    out = sim("Exercise2.slx");

    figure('Name',sprintf('sigma_r_%g',sigma_r),'NumberTitle','off')
    subplot(2,1,1)
    title(sprintf('h_b = %g - K_b = %g',hb,Kb))
    yyaxis left
    plot(out.Speed.time, out.Speed.signals.values*3.6)
    xlabel('Time [s]')
    ylabel('Speed [km/h]')
    yyaxis right
    plot(out.Distance.time, out.Distance.signals.values)
    ylabel('Distance [m]')

    subplot(2,1,2)
    title(sprintf('\\sigma_r = %g - K_{gain} = %g',sigma_r,K_gain))
    hold on;
    plot(out.slip.time, out.slip.signals(1).values)
    plot(out.slip.time, out.slip.signals(2).values)
    plot(out.slip.time, out.slip.signals(3).values)
    plot(out.slip.time, out.slip.signals(4).values)
    yline(sigma_r,'--k','LineWidth',1)
    legend('Front Left tyre','Front Right tyre','Rear Left tyre','Rear Right tyre','\sigma_r')
    xlabel('Time [s]')
    ylabel('Slip [-]')
    ylim([-1 1])
end
% return

% % 3) Tune the value of $K$keeping the value of $\sigma_r$ at -0.2 to achieve minimum stopping distance

K_gain = -1450;
sigma_r = -0.2;
out = sim("Exercise2.slx");

figure('Name','Tune_K_gain','NumberTitle','off')
subplot(2,1,1)
title(sprintf('h_b = %g - K_b = %g',hb,Kb))
hold;
yyaxis left
plot(out.Speed.time, out.Speed.signals.values*3.6)
xlabel('Time [s]')
ylabel('Speed [km/h]')
yyaxis right
plot(out.Distance.time, out.Distance.signals.values)
ylabel('Distance [m]')

subplot(2,1,2)
title(sprintf('\\sigma_r = %g - K_{gain} = %g',sigma_r,K_gain))
hold on;
plot(out.slip.time, out.slip.signals(1).values)
plot(out.slip.time, out.slip.signals(2).values)
plot(out.slip.time, out.slip.signals(3).values)
plot(out.slip.time, out.slip.signals(4).values)
legend('Front Left tyre', 'Front Right tyre', 'Rear Left tyre', 'Rear Right tyre')
xlabel('Time [s]')
ylabel('Slip [-]')
ylim([-1 1])
% return

%% 4) Repeat point 3 assuming wet conditions (modify the $\mu -\sigma {\;}_r$ 
% block accordingly) and discuss the different results.  

VehicleParameters;
sigma_r    = -0.2;
Kb         = 2.33;
hb         = 0.5;
K_gain_dry = -1450;     % tuned in point 3 (dry)
K_gain_wet = -700;     % re-tune here for wet condition

% --- DRY simulation ---
mu_vec  = mu_dry;
K_gain  = K_gain_dry;
out_dry = sim("Exercise2.slx");

% --- WET simulation ---
mu_vec  = mu_wet;
K_gain  = K_gain_wet;
out_wet = sim("Exercise2.slx");

figure('Name','Dry_vs_Wet_road_conditions','NumberTitle','off')

% Speed/Distance - DRY (top-left)
subplot(2,2,1)
title(sprintf('DRY - h_b = %g - K_b = %g',hb,Kb))
yyaxis left
plot(out_dry.Speed.time, out_dry.Speed.signals.values*3.6)
xlabel('Time [s]')
ylabel('Speed [km/h]')
yyaxis right
plot(out_dry.Distance.time, out_dry.Distance.signals.values)
ylabel('Distance [m]')

% Speed/Distance - WET (top-right)
subplot(2,2,2)
title(sprintf('WET - h_b = %g - K_b = %g',hb,Kb))
yyaxis left
plot(out_wet.Speed.time, out_wet.Speed.signals.values*3.6)
xlabel('Time [s]')
ylabel('Speed [km/h]')
yyaxis right
plot(out_wet.Distance.time, out_wet.Distance.signals.values)
ylabel('Distance [m]')

% Slip - DRY (bottom-left)
subplot(2,2,3)
title(sprintf('DRY - \\sigma_r = %g - K_{gain} = %g',sigma_r,K_gain_dry))
hold on;
plot(out_dry.slip.time, out_dry.slip.signals(1).values)
plot(out_dry.slip.time, out_dry.slip.signals(2).values)
plot(out_dry.slip.time, out_dry.slip.signals(3).values)
plot(out_dry.slip.time, out_dry.slip.signals(4).values)
yline(sigma_r,'--k','LineWidth',1)
legend('Front Left tyre','Front Right tyre','Rear Left tyre','Rear Right tyre','\sigma_r')
xlabel('Time [s]')
ylabel('Slip [-]')
ylim([-1 1])

% Slip - WET (bottom-right)
subplot(2,2,4)
title(sprintf('WET - \\sigma_r = %g - K_{gain} = %g',sigma_r,K_gain_wet))
hold on;
plot(out_wet.slip.time, out_wet.slip.signals(1).values)
plot(out_wet.slip.time, out_wet.slip.signals(2).values)
plot(out_wet.slip.time, out_wet.slip.signals(3).values)
plot(out_wet.slip.time, out_wet.slip.signals(4).values)
yline(sigma_r,'--k','LineWidth',1)
legend('Front Left tyre','Front Right tyre','Rear Left tyre','Rear Right tyre','\sigma_r')
xlabel('Time [s]')
ylabel('Slip [-]')
ylim([-1 1])

%% 
% 5) Discuss whether or not the control presented in this exercise can be installed 
% on a vehicle or not and its practical relevance.
%% Exercise 3. ABS - Conjugate Boundary Method

% K_release = ;
% K_reapply = ;
% K1 = ;
% K2 = ;
% K3 = ;


%