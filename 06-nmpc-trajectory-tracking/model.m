

function [dxdt,y] = model(x,u)
    % Main parameters
    l1=1.2;     % m
    l2=1.6;     % m
    m=1575;     % kg
    Jz=4000;     % kg*m^2
    C1=2.7e4;   % N/rad
    C2=2e4;     % N/rad
    
    X         = x(1);
    Y         = x(2);
    psi       = x(3);
    vx        = x(4);
    vy        = x(5);
    omega_psi = x(6);
    ax = u(1);
    delta = u(2);

    beta1 = atan((vy + l1 * omega_psi) / vx) - delta;
    beta2 = atan((vy - l2 * omega_psi) / vx);
    Fy1 = -C1 * beta1 * cos(delta);
    Fy2 = -C2 * beta2;
    dX_dt         = vx * cos(psi) - vy * sin(psi);
    dY_dt         = vx * sin(psi) + vy * cos(psi);
    dpsi_dt       = omega_psi;
    dvx_dt        = vy * omega_psi + ax;
    dvy_dt        = -vx * omega_psi + (1 / m) * (Fy1 + Fy2);
    domega_psi_dt = (1 / Jz) * (l1 * Fy1 - l2 * Fy2);
    y = [X; Y; beta1 ];
    dxdt = [dX_dt; dY_dt; dpsi_dt; dvx_dt; dvy_dt; domega_psi_dt];
end











