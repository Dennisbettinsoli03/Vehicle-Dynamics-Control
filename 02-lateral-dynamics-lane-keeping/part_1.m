%% Project 2 - part 1 - Lateral Dynamics 
%  Single-track (bicycle) linear model. States x = [beta; psi_dot].

clear; clc; close all;

% % Vehicle parameters
a  = 0.923;     % front axle - c.o.g. distance [m]
b  = 1.376;     % rear  axle - c.o.g. distance [m]
m  = 1020;      % vehicle mass [kg]
Jz = 1225;      % yaw inertia [kg m^2]
t  = 1.365;     % track width [m]
g  = 9.81;

%% Cornering stiffness C1, C2 from the tire file
% The Fy-vs-alpha block sits in columns Q:V, data rows 3:203.
% Col 1 = alpha [deg]; cols 2..6 = -Fy [N] for Fz = 2,4,6,8,10 kN.
T = readmatrix('TireData.xlsx','Sheet','Tire - Pacejka model data', ...
               'Range','Q3:V203');
alpha_deg = T(:,1);
alpha     = deg2rad(alpha_deg);
Fy        = -T(:,2:6);                 % real lateral force (remove the minus)

% Static axle loads
Fz1 = m*g*b/(a+b);   
Fz2 = m*g*a/(a+b);
fprintf('Static axle loads:  front = %g N   rear = %g N\n',Fz1,Fz2);

% Cornering stiffness = -dFy/dalpha at alpha=0, via linear fit near zero.
idx = abs(alpha_deg) < 2;              % points close to alpha = 0
slope = @(col) polyfit(alpha(idx), Fy(idx,col), 1);
p6 = slope(3);   Cp_front = -p6(1);    % per tire, Fz = 6 kN  (column 3)
p4 = slope(2);   Cp_rear  = -p4(1);    % per tire, Fz = 4 kN  (column 2)

% Per-axle stiffness (two tires per axle) -> bicycle-model convention
C1 = 2*Cp_front;
C2 = 2*Cp_rear;
fprintf('C1 (front axle) = %.0f N/rad\nC2 (rear axle)  = %.0f N/rad\n\n',C1,C2);

%% 1) Stability derivatives and state-space at 80 km/h
V0 = 80/3.6;                           % [m/s]
[A0,B0,der] = bicycleSS(V0,C1,C2,a,b,m,Jz);

fprintf('--- Stability derivatives at 80 km/h ---\n');
fprintf('Y_beta = %.1f  Y_r = %.1f  N_beta = %.1f  N_r = %.1f  Y_delta = %.1f  N_delta = %.1f\n',...
        der.Yb,der.Yr,der.Nb,der.Nr,der.Yd,der.Nd);
disp('A ='); disp(A0);
disp('B ='); disp(B0);
fprintf('Poles at 80 km/h: %s\n\n', mat2str(eig(A0),4));

%% 2) Root locus vs speed 
Vrange = 2:0.5:60;                     % [m/s]
poles  = zeros(2,numel(Vrange));

for k = 1:numel(Vrange)
    Ak = bicycleSS(Vrange(k),C1,C2,a,b,m,Jz);
    poles(:,k) = eig(Ak);
end

figure('Name','Root loci vs speed','NumberTitle','off')
grid minor, hold on, box on
plot(real(poles).',imag(poles).','x','MarkerSize',8)
xlabel('Real')
ylabel('Imag')
title('Root loci of [\beta, \psi-dot] state space system')

%% 3) Step steer at 80 km/h 
C = eye(2); D = zeros(2,1);
sys = ss(A0,B0,C,D);
delta0 = 0.03;                         % step steer amplitude [rad]
tspan  = 0:0.001:3;
y = step(sys,tspan) * delta0;          % y(:,1)=beta, y(:,2)=psi_dot

figure('Name','Step steer at 80 km/h','NumberTitle','off')
subplot(2,1,1)
plot(tspan,y(:,1),'LineWidth',1.3)
grid minor
ylabel('\beta  [rad]')
title(sprintf('Step steer \\delta = %.3f rad @ 80 km/h',delta0))

subplot(2,1,2)
plot(tspan,y(:,2),'LineWidth',1.3)
grid minor
ylabel('\psi-dot  [rad/s]'); xlabel('time [s]')

%% 4) Steady-state gains vs speed
curv = zeros(size(Vrange));            % path curvature gain  1/(R*delta)
ay   = zeros(size(Vrange));            % lateral acceleration gain  V^2/(R*delta)
sideSlip_gain  = zeros(size(Vrange));            % sideslip angle gain  beta/delta

for k = 1:numel(Vrange)
    V = Vrange(k);
    [~,~,d] = bicycleSS(V,C1,C2,a,b,m,Jz);
    den = d.Nb*(m*V - d.Yr) + d.Nr*d.Yb;
    num = d.Yd*d.Nb - d.Nd*d.Yb;
    curv(k) = num / (V*den);
    ay(k)   = V*num / den;
    sideSlip_gain(k)  = (-d.Nd*(m*V - d.Yr) - d.Nr*d.Yd) / den;
end

figure
sgtitle('Steady-State Gains vs Speed')
subplot(3,1,1)
plot(Vrange*3.6,curv,'r-','LineWidth',1.3)
title('Curvature gain')
xline(V0*3.6,'r--')
xlim([0 Vrange(end)*3.6])
grid minor, box on
ylabel('1/(R\delta)')


subplot(3,1,2)
plot(Vrange*3.6,ay,'b-','LineWidth',1.3)
title('Lateral acceleration gain')
xline(V0*3.6,'r--')
grid minor, box on
xlim([0 Vrange(end)*3.6])
ylabel('V^2/(R\delta)')

subplot(3,1,3)
plot(Vrange*3.6,sideSlip_gain,'k-','LineWidth',1.3)
title('Side slip angle gain')
xline(V0*3.6,'r--')
grid minor, box on
xlim([0 Vrange(end)*3.6])
ylabel('\beta/\delta')
xlabel('speed [km/h]')

%% Local function: build state-space at a given speed
function [A,B,der] = bicycleSS(V,C1,C2,a,b,m,Jz)
    der.Yb = -(C1 + C2);
    der.Yr = -(a*C1 - b*C2)/V;
    der.Nb = -(a*C1 - b*C2);
    der.Nr = -(a^2*C1 + b^2*C2)/V;
    der.Yd = C1;
    der.Nd = a*C1;
    A = [ der.Yb/(m*V),  der.Yr/(m*V)-1 ;
          der.Nb/Jz,     der.Nr/Jz      ];
    B = [ der.Yd/(m*V) ;
          der.Nd/Jz    ];
end
