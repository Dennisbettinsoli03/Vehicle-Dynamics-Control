clear
close all
clc

%% Speed setting
km_h = input("Set km/h of the vehicle (65 km/h or 100 km/h): ");

if km_h == 65
    disp("OKAY!")
    distance = 4000;
elseif km_h == 100
    disp("OKAY!")
    distance = 6000;
else
    fprintf("Incorrect value of km_h: %.2f\n"+ ...
        "Retry...\n",km_h);
    return
end

%% Reference trajectory
Xr=linspace(0,distance,20000)';
Yr=10*sin(0.01*Xr);
psir=[atan2(diff(Yr),diff(Xr));0];
refPoses=[Xr,Yr,psir];

%% PID design
% Linearization point
Xl=0;
Yl=0;
psil=psir(1);
vxl=km_h/3.6;     % Speed assumed for controller design
vyl=0;
wpsil=0;
zel=[Xl;Yl;psil;vxl;vyl;wpsil];

%% Simulation

% Initial conditions. 
X0=0;
Y0=0;
psi0=0;
vx0=80/3.6;     % Speed assumed for controller design
vy0=0;
wpsi0=0;
ze0=[X0;Y0;psi0;vx0;vy0;wpsi0];

%% Simulation time & PID tuning
Tfin=150;

%% Automated simulation
nfile1='project_2_part2_ex1.slx';
% open(nfile1)
sim(nfile1)

% EXERCISE 1
figure('Name',sprintf('Exercise 1: delta_f'),'NumberTitle','off')
plot(delta.Time,delta.Data,'r-','LineWidth',1.5)
grid on, box on
xlabel('time (s)')
ylabel(sprintf('\\delta (deg)'))
title('Steering angle','FontSize',15)

figure('Name',sprintf('Exercise 1: DST & DSTP'),'NumberTitle','off')
subplot(2,2,1)
plot(squeeze(X_linear.Data),squeeze(Y_linear.Data),'b.-','LineWidth',0.5)
grid on, box on
title('Linear vehicle''s model')
xlabel('X position (m)')
ylabel('Y position (m)')

subplot(2,2,2)
plot(vx_linear.Time,squeeze(vx_linear.Data),'k-','LineWidth',1)
grid on, box on
title('Vx linear model')
xlabel('time (s)')
ylabel('Vx (m/s)')

subplot(2,2,3)
plot(squeeze(X_pacejka.Data),squeeze(Y_pacejka.Data),'b.-','LineWidth',0.5)
grid on, box on
title('Pacejka vehicle''s model')
xlabel('X position (m)')
ylabel('Y position (m)')

subplot(2,2,4)
plot(vx_pacejka.Time,squeeze(vx_pacejka.Data),'k-','LineWidth',1)
grid on, box on
title('Vx Pacejka model')
xlabel('time (s)')
ylabel('Vx (m/s)')

% Parameters for figures
set_controller = [2 1];
set_controller2 = [2 1];
controller = {'PDF','PIDF'};

for i = 1:2
    controller1 = set_controller(i);
    controller2 = set_controller2(i);

    nfile1='project_2_part2_ex2.slx'; 
    sim(nfile1)

    % % FIGURES
    % EXERCISE 2
    figure('Name',sprintf('Exercise 2: ref. trajectory + %s',controller{i}),'NumberTitle','off')
    subplot(3,1,1)
    plot(refPoses(:,1),refPoses(:,2),'k-','LineWidth',0.5)
    hold on, box on, grid on
    plot(X_a.Data,Y_a.Data,'r--','LineWidth',0.5)
    xlabel('X position (m)')
    ylabel('Y position (m)')
    xlim([0 X_a.Data(end)])
    title(sprintf('e_c_t - %d km/h - %s',km_h,controller{i}), 'FontSize', 15);
    legend('Ref. trajectory',['',controller{i}])

    subplot(3,1,2)
    plot(e_ct.Time,e_ct.Data,'r-','LineWidth',1)
    box on, grid on
    xlabel('time (s)')
    ylabel('cross-track error [m]')
    title('e_c_t','FontSize',15)

    subplot(3,1,3)
    plot(e_h.Time,e_h.Data,'r-','LineWidth',1)
    box on, grid on
    xlabel('time (s)')
    ylabel('heading error [deg]')
    title('e_h','FontSize',15)

    figure('Name',sprintf('Exercise 2: ref. trajectory + %s',controller{i}),'NumberTitle','off')
    subplot(3,1,1)
    plot(refPoses(:,1),refPoses(:,2),'k-','LineWidth',0.5)
    hold on, box on, grid on
    plot(X_a2.Data,Y_a2.Data,'r--','LineWidth',0.5)
    xlabel('X position (m)')
    ylabel('Y position (m)')
    xlim([0 X_a2.Data(end)])
    title(sprintf('e_c_t + e_h - %d km/h - %s',km_h,controller{i}), 'FontSize', 15);
    legend('Ref. trajectory',['',controller{i}])

    subplot(3,1,2)
    plot(e_ct2.Time,e_ct2.Data,'r-','LineWidth',1)
    box on, grid on
    xlabel('time (s)')
    ylabel('cross-track error [m]')
    title('e_c_t','FontSize',15)

    subplot(3,1,3)
    plot(e_h2.Time,e_h2.Data,'r-','LineWidth',1)
    box on, grid on
    xlabel('time (s)')
    ylabel('heading error [deg]')
    title('e_h','FontSize',15)

    figure('Name','Exercise 2: Steering Actuator','NumberTitle','off')
    subplot(2,1,1)
    plot(delta_f.Time, delta_f.Data,'g-','LineWidth',1)
    box on, grid on, hold on
    xlabel('time (s)')
    ylabel(sprintf('\\delta_f [deg]'))
    title(['Steering angle - e_c_t - ',controller{i}],'FontSize',15)

    subplot(2,1,2)
    plot(delta_f2.Time, delta_f2.Data,'g-','LineWidth',1)
    box on, grid on
    xlabel('time (s)')
    ylabel(sprintf('\\delta_f [deg]'))
    title(['Steering angle - e_c_t + e_h - ',controller{i}],'FontSize',15)
end

% EXERCISE 3 - STANLEY
nfile2='project_2_part2_ex3.slx';
sim(nfile2)

figure('Name',sprintf('Exercise 3: ref. trajectory + %s & Stanley',controller{i}),'NumberTitle','off')
subplot(3,1,1)
plot(refPoses(:,1),refPoses(:,2),'k-','LineWidth',0.5)
hold on, box on, grid on
plot(X_a.Data,Y_a.Data,'r--','LineWidth',0.5)
plot(X_a_stanley.Data,Y_a_stanley.Data,'c--','LineWidth',0.5)
xlabel('X position (m)')
ylabel('Y position (m)')
xlim([0 X_a.Data(end)])
title(sprintf('e_c_t - %d km/h - %s',km_h,controller{i}), 'FontSize', 15);
legend('Ref. trajectory',['',controller{i}],'Stanley')

subplot(3,1,2)
plot(e_ct.Time,e_ct.Data,'r-','LineWidth',1)
hold on, box on, grid on
plot(e_ct_stanley.Time,e_ct_stanley.Data,'c-','LineWidth',1)
xlabel('time (s)')
ylabel('cross-track error [m]')
title('e_c_t','FontSize',15)
legend(['',controller{i}],'Stanley')

subplot(3,1,3)
plot(e_h.Time,e_h.Data,'r-','LineWidth',1)
hold on, box on, grid on
plot(e_h_stanley.Time,e_h_stanley.Data*(180/pi),'c-','LineWidth',1)
xlabel('time (s)')
ylabel('heading error [deg]')
title('e_h','FontSize',15)
legend(['',controller{i}],'Stanley')

curr_figs = findall(0, 'type', 'figure');

namefig = sprintf('figures_%d_km_h.fig',km_h);
savefig(curr_figs,namefig)
close all