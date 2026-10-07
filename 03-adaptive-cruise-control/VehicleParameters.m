%% vehicle data from Genta vol.2 The automotive chassis Appendix E.1
m      = 1020;  % mass [kg]
Je     = 0.088; % engine inertia [kg*m^2]
% in the parameters from Genta book, it is indicated as Jm (motore)
Jt     = 0.05;  % transmission inertia [kg*m^2]
Jw     = 0.59;  % single wheel inertia [kg*m^2]
% in the parameters from Genta book, it is indicated as Jr (ruota)
Jwf    = Jw;
Jwr    = Jw;
i_g    = [3.909 , 2.157 , 1.48 , 1.121 , 0.897]; % gearbox transmission ratio
tau_g  = 1./i_g;
ratio  = (i_g(1)/i_g(end))^(1/5);
ratio_longer = (i_g(1)/i_g(end))^(1/4);
i_g_new      = [i_g(end)*ratio^5, i_g(end)*ratio^4,i_g(end)*ratio^3,i_g(end)*ratio^2,i_g(end)*ratio, i_g(end)];
i_g_longer   = [i_g(end)*ratio_longer^4, i_g(end)*ratio_longer^3,i_g(end)*ratio_longer^2,i_g(end)*ratio_longer,i_g(end)*ratio_longer^-1, i_g(end)*ratio_longer^-2];
i_g_shorter  = flip([i_g(1)/ratio_longer^4, i_g(1)/ratio_longer^3,i_g(1)/ratio_longer^2,i_g(1)/ratio_longer,i_g(1)/ratio_longer^0, i_g(1)/ratio_longer^-1]);
i_f    = 3.438; % driveline transmission ratio
tau_f  = 1./i_f;
eta_t  = 0.93; % total transmission efficieny
% a = 1.376;
% b = 0.923;
a      = 0.923; % front axle-c.o.g. distance [m]
b      = 1.376; % rear axle-c.o.g. distance [m]
l      = a+b;   % wheelbase [m]
hG     = 0.6;   % c.o.g. height [m]
S      = 2.04;  % resistant area [m^2]
Cx     = 0.33;  % drag coefficient
Cz     = 0.2;   % lift coefficient
CMy    = 0.1;   % pitching moment coefficient
ro     = 1.2;   % dry air density at 20°C [kg/m^3]
f0     = 0.011; 
K      = 2.6e-8; % [s^2/m^2]
Re     = 0.279;  % rolling radius [m]
Rc     = Re;     % first approximation
n      = 2;      % number of driving wheels
n_nd   = n;      % number of non-driving wheels
C_sigmaFz = 5;   % nominal longitudinal tire stiffness
Kb        = 2.3; % braking 70/30
Tbf_max   = 300; % max front braking torque [N*m]
Te_max    = 105; % max engine torque [N*m]
Tc_max    = 1.2*Te_max; % max clutch torque [N*m]
%% resistance force
g     = 9.81; % gravity acceleration [m/s^2]
alpha = 0;    % road grade [rad]
A     = m*g*(f0*cos(alpha)+sin(alpha));
B     = m*g*K*cos(alpha)+0.5*ro*S*(Cx-Cz*f0);
C     = -0.5*ro*S*K*Cz;

%% inertia calculations
i     = 3; % gear selection

%% engine parameters
n_baseline = linspace(85, 700, 15);
T_baseline = [30, 53, 70, 76, 85, 88, 90, 91, 90, 87, 82, 72, 65, 40, -50];
n_compact  = [1000:200:5400 5500 5550 5600 5700]*pi/30;
T_compact  = [78 84 89 93 94 98 101 102.5 103.9 104 103 102.5 102 100 99 96.5 94 89 87.5 84 81.9 78 73.5 72 30 -10 -10];

n_saloon = [1000:100:4500 4501 4502 4503 4504 4505 4506 4550 4600 4700]*pi/30;
T_saloon = [76 118 151 180 199 215 238 250 262 270 273 277 276 276 275 272 270 265 260 254 249 242 238 232 229 222 219 215 211 208.5 202 198.5 190 185 178 169 168 167 160 80 79 78 20 -30 -30];

n_van    = [750, 800, 1000, 1250, 1500, 1750, 2000, 2250, 2500, 2750, 3000, 3250, 3500, 3750, 4000, 4250]*pi/30;
T_van    = [50, 80, 216.9, 314.6, 379.9, 390.4, 386.4, 380.8, 368.8, 353.9, 335.3, 333.6, 313.2, 245.1, 147.2, -10];

n_BEV    = [500:250:10000, 10250]*pi/30;
T_BEV    = [287.32 287.23 287.15 287.07 286.99 286.91 286.82 286.74 267.38 243.07 222.81 205.67 190.98 178.25 167.11 157.28 148.54 140.73 133.69 127.32 121.54 116.25 111.41 106.95 102.84 99.03 95.49 92.20 89.13 86.25 83.56 81.02 78.64 76.39 74.27 72.26 70.36 68.56 66.84 -10];

Traction = 1;

%% Shifting parameters

t_pedalAct = 0.8;
t_shifting = 0.2;