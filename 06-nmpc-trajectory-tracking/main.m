clear all
close all
clc

global Co

%% Vehicle parameters

% Main parameters
l1=1.2;     % m
l2=1.6;     % m
m=1575;     % kg
Jz=4000;     % kg*m^2
C1=2.7e4;   % N/rad
C2=2e4;     % N/rad

% Tire model
load tire_model

%% Global trajectory planning 

rp=3;
switch rp 
    case 1  % real road
        load real_road_1
        Xr=Xr(1:3e3);
        Yr=Yr(1:3e3);
        vx_ref=70/3.6;
        Tfin=25;
    case 2  % sinusoidal road
        ii=linspace(0,1000,10000)';
        Xr=6*sin(0.05*ii);
        Yr=ii;
        vx_ref=60/3.6;
        Tfin=20;   
    case 3  % straight road with obstacle
        vx_ref=70/3.6;
        Xr=linspace(0,350,10000)';
        Yr=0.3*Xr;
        Co=[132;38]; 
        Ro=2;
        Tfin=15;
end

psir=[atan2(diff(Yr),diff(Xr));0];
refPoses=[Xr,Yr,psir];

%% NMPC design

par.model = @(t, x, u) model(x, u);
par.nlcon = @nlcon;
par.nx=6;

par.Ts= 0.05;
par.Tp= 3;

par.R= diag([0.01, 0.1]);
par.Q= diag([1, 1, 12]);

par.lb=  [-5, -0.4];
par.ub=  [3, 0.4];

K=nmpc_design_st2(par,2);

% Number of points of the reference trajectory portion corresponding
% to the time interval [t t+Tp] at the speed vx_ref.
dX=diff(refPoses(:,1));
dY=diff(refPoses(:,2));
ds=sqrt(dX.^2+dY.^2);
Np=round(vx_ref*K.Tp/mean(ds));

%% Simulation

% Initial conditions;
X0=refPoses(1,1);
Y0=refPoses(1,2);
psi0=refPoses(1,3);
vx0=vx_ref;
vy0=0;
wpsi0=0;
ze0=[X0;Y0;psi0;vx0;vy0;wpsi0];

% Other simulation parameters
Ts=0.05;    
X0=refPoses(1,1);
Ta=0.05;
t = 0:Ts:Tfin;

% Simulation
open('sim_aut_dual_scen');
sim('sim_aut_dual_scen');
delta = delta.Data;
ax = ax.Data;
u = [ax delta];
%%
figure('Name','Base trajectory'); hold on;
title('Base trajectory');
xlabel('X [m]');
ylabel('Y [m]');
plot(ze.Data(:,1),ze.Data(:,2))

figure('Name','Controls','Position',[100 100 800 400]);
subplot(2,1,1);
plot(t(1:length(ax)), ax, 'b','LineWidth',1.2);
grid on;
ylabel('Long. Accel. a_x [m/s^2]');
title('Control: Longitudinal Acceleration');

subplot(2,1,2);
plot(t(1:length(delta)), delta, 'r','LineWidth',1.2);
grid on;
ylabel('Steering Angle \delta [rad]');
xlabel('Time [s]');
title('Control: Steering Angle');


% Plot lateral/heading errors and cross-track error
figure('Name','Errors','Position',[200 200 800 600]);
subplot(2,1,1);
if size(errors,2)>=1
    plot(errors(:,1),'b','LineWidth',1.2);
    ylabel('e_x [m]');
    grid on;
    title('Longitudinal Error (e_x)');
end

subplot(2,1,2);
if exist('e_ct','var') && ~isempty(e_ct)
    plot( e_ct, 'y','LineWidth',1.2);
    ylabel('e_{ct} [m]');
    xlabel('Time [s]');
    grid on;
    title('Cross-Track Error (e_{ct})');
end

%% Create road scenario
% road_scenario

