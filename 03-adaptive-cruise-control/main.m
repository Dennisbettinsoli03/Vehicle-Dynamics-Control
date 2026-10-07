clear
close all
clc
Ts = 0.01;
load LUT_values.mat

% Load vehicle parameters;
VehicleParameters;

% Select the vehicle type
vec_choice = input('Select the vehicle type: 1 = saloon; 2 = van; 3 = compact; 4 = baseline; 5 = BEV \n');

if vec_choice < 1 || vec_choice > 5
    error('Invalid vehicle type selected (%d). Please choose a number between 1 and 5.', vec_choice);
end

% Angular speed dataset
n_vec_set = {n_saloon, n_van, n_compact, n_baseline, n_BEV};
% Torque dataset
T_vec_set = {T_saloon, T_van, T_compact, T_baseline, T_BEV};

n_vec = n_vec_set{vec_choice};
T_vec = T_vec_set{vec_choice};

% ABS
sigma_r = -0.2;
K_gain = -1200;

% Initial conditions
dist0 = 100;
v0 = 30;
omega0 = v0/Re;
startingGear = 3;

% gear shifting:
n_shiftup   = 500;
n_shiftdown = 200;
sigma_vec   = sigma;
gear_vec    = i_g;

road_cond = {mu_dry mu_wet};
choice = input('Select the road condition: 1 = dry 2 = wet \n');
if choice ~= 1 && choice ~= 2
    warning('Wrong choice. Reset to dry condition');
    choice = 1;
end
mu_vec = road_cond{choice};

%% ABS parameters
lower_acc   = -80;
lower_slip  = -0.17;
higher_acc  = 500;
rise_rate   = 50*Ts;
fall_rate   = 35*Ts;
mean_acc    = 50;

ABS_parameters = [lower_acc; lower_slip; higher_acc; mean_acc; rise_rate; fall_rate];

%% ACC Parameters
h      = 2.7;
lambda = 0.8;

a_max_acc  =  2.0;
a_max_dec  = -3.5;
v_min_acc  =  5.0;
v_set_min  =  7.0;
max_dec_rate = -2.5;

tau_virtual = 0.2;
kp = 1/3;

% FIR filter coefficients (2-second moving average)
fir_window = round(2/Ts);
coeff = ones(1, fir_window) / fir_window;

% PI parameters
P = kp;
I = kp / tau_virtual;

open("DASD_B_Project_3.slx");
out = sim("DASD_B_Project_3.slx");

%% Import data from Simulink
t = out.tout;
ha = out.ha.Data;
hb = out.hb.Data;
time_gap = out.time_gap.Data;
a_target = out.a_target.Data;
v_leader = out.v_leader.Data;
vdot = out.vdot.Data;
e_acc = out.acc_error.Data;
x_ego = out.x_ego.Data;
v_ego = out.v_ego.Data;
x_leader = out.x_leader.Data;

% ---------------------------------------------------------
% Figure 1: Vehicle Control Inputs (Pedals and Gear)
% ---------------------------------------------------------
fig1 = figure('Name', 'Vehicle Control Inputs', 'Color', 'w');

ax1 = subplot(2,1,1);
plot(t, hb, 'r', 'LineWidth', 1.5);
grid on; grid minor;
ylabel('Brake Position');
title('Driver / Controller Inputs Over Time', 'FontSize', 12);
ylim([-0.05 1.05]);

ax2 = subplot(2,1,2);
plot(t, ha, 'Color', 'c', 'LineWidth', 1.5);
grid on; grid minor;
ylabel('Accel Position');
ylim([-0.05 1.05]);

linkaxes([ax1, ax2], 'x');

% ---------------------------------------------------------
% Figure 2: ACC Metrics (Acceleration and Time Gap)
% ---------------------------------------------------------
fig2 = figure('Name', 'ACC Metrics', 'Color', 'w');

ax4 = subplot(3,1,1);
plot(t, a_target, 'LineWidth', 1.5, 'Color', 'b'); hold on
plot(t, vdot, 'LineWidth', 1.5, 'Color', 'g');
grid on; grid minor;
ylabel('Acceleration (m/s^2)');
title('Adaptive Cruise Control Metrics', 'FontSize', 12);
legend('Commanded', 'Actual', 'Location', 'best');
ylim([min(a_target)-1, max(a_target)+1]);

ax5 = subplot(3,1,2);
plot(t, e_acc, 'LineWidth', 1.5, 'Color', 'k');
grid on; grid minor;
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Accel error (m/s^2)');

ax6 = subplot(3,1,3);
plot(t, time_gap, 'LineWidth', 1.5, 'Color', 'g');
grid on; grid minor;
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Time gap (s)');
ylim([0, max(time_gap)*1.1]);
yline(h, '--r', 'Target', 'LineWidth', 1);

linkaxes([ax4, ax5, ax6], 'x');

% ---------------------------------------------------------
% Figure 3: Leader vs Follower comparison
% ---------------------------------------------------------
fig3 = figure('Name', 'Leader vs Follower');
ax7 = subplot(2,1,1);
plot(t, x_ego, 'LineWidth', 1.5, 'Color', 'g'); hold on
plot(t, x_leader, 'LineWidth', 1.5, 'Color', 'b');
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Position (m)', 'FontWeight', 'bold');
title('Vehicle Positions and Speeds');
grid on; grid minor;
legend('Ego', 'Leader', 'Location', 'best');

ax8 = subplot(2,1,2);
plot(t, v_ego, 'LineWidth', 1.5, 'Color', 'r'); hold on
plot(t, v_leader, 'LineWidth', 1.5, 'Color', 'c');
xlabel('Time (s)', 'FontWeight', 'bold');
ylabel('Speed (m/s)', 'FontWeight', 'bold');
legend('Ego', 'Leader', 'Location', 'best');
grid on; grid minor;
linkaxes([ax7, ax8], 'x');
