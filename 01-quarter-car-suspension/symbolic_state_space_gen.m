%% Quarter car with skyhook and groundhook -
%%      equations 27.80 and 27.81
% Use this code to obtain the state space matrices for the quarter car model with skyhook and groundhook forces included

clc
close all
clear

syms Zs Vs Zu Vu r drdt real                 % States and input
syms ms mu K P zeta0 zeta_d zeta_t real  % System parameters
syms a_s a_u real                            % Accelerations (second derivatives)

% Mass matrix
M = diag([ms, mu]);

% Damping matrix
D = 2 * sqrt(K*ms) * [ zeta0 + zeta_d, -zeta0 + zeta_d;
                      -zeta0 - zeta_d,  zeta0 + zeta_t - zeta_d];

% Stiffness matrix
Kmat = [ K,   -K;
        -K, K + P ];

% RHS forcing vector
F = [ 0;
      2 * sqrt(K*ms) *zeta_t*drdt + P*r ]; % c_t*drdt + P*r ]

% Build symbolic system of equations: M*[a_s; a_u] + D*[Vs; Vu] + K*[Zs; Zu] = F
eqns = M * [a_s; a_u] + D * [Vs; Vu] + Kmat * [Zs; Zu] - F;

% Solve for accelerations
sol = solve(eqns == 0, [a_s, a_u]);

% Build state vector x = [Zs; Vs; Zu; Vu; h]
% and its derivative dx = [dZs; dVs; dZu; dVu; dh]
dx = sym('dx', [5 1]);

dx(1) = Vs;                   % dZs/dt = Vs
dx(2) = simplify(sol.a_s);   % dVs/dt = a_s
dx(3) = Vu;                   % dZu/dt = Vu
dx(4) = simplify(sol.a_u);   % dVu/dt = a_u
dx(5) = drdt;                % dh/dt = dhdt (input)

% Final symbolic state-space representation
disp('State vector x = [Zs; Vs; Zu; Vu; r]')
disp('Input u = drdt')
disp('State derivatives dx = f(x,u):')
disp(dx)
%% state space representation

% Define symbolic variables again for state and input if not already
x = [Zs; Vs; Zu; Vu; r];
u = drdt;

% Compute Jacobians
A = jacobian(dx, x);  % df/dx
B = jacobian(dx, u);  % df/du

% Output equation: y = Cx + Du
% Example: output is [Zs_dotdot vertical_load  actuator_force]
y = [sol.a_s; ...                                       sprung vertical acceleration
     P*(r - Zu) + 2 * sqrt(K*ms) *zeta_t*drdt;...  dynamic vertical_load % P*(h - Zu) + c_t*dhdt
     -2*sqrt(K*ms)*zeta_d*(Vs + Vu);...                    actuator force from eq. (27.85)
     ];

% Fact = -2*sqrt(K*ms)*(zeta_s + zeta)*Vs + 2*sqrt(K*ms)*(zeta - zeta_g)*Vu 

C = jacobian(y, x);   % dy/dx
D = jacobian(y, u);   % dy/du → zero in this case

%% output is [Zs_dotdot; Ft; F_act]


% Display results
disp('State matrix A:'); disp(simplify(A))
disp('Input matrix B:'); disp(simplify(B))
disp('Output matrix C:'); disp(C)
disp('Feedthrough matrix D:'); disp(D)