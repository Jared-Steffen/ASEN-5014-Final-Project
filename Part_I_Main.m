clc; clear; close all

%% OL SS System

% Constants
mu = 3.986e5; % km^3/s^2
r0 = 6678; % km

% ICs
delta_x0 = [0.001 -5e-6 0 5e-6]';
delta_x0_int_control = [0.01 -0.005 0 0.005 0 0]';

% Linearized SS System
A = [0 1 0 0;
     (3*mu)/r0^3 0 0 2*sqrt(mu/r0);
     0 0 0 1;
     0 -2*sqrt(mu/r0^5) 0 0];

B = [0 0;
     1 0;
     0 0;
     0 1/r0];

C = [1 0 0 0;
     0 0 1 0];

D = [0 0;
     0 0];

%% Thruster Max/Reference Input Profile
% Time Vector
tend = 60;
tspan = 0:0.1:tend;

% Starlink V2 Mini Thruster Profile
SLV2m_thrust = 0.17; % N
SC_mass = 250; % kg (estimate)
SLV2m_accel = SLV2m_thrust/SC_mass;
u_max = SLV2m_accel;

rt = 0.05*sign(double(tspan > 10)); % Begin burn maneuver at 10s

%% Disturbance Profile
% Constant Bias (due to Drag?)
dmag = -1e-6;
dt = dmag.*ones(length(tspan),1);

%% Determine OL Poles and Augment for Disturbances
% OL Poles
V = eig(A);

% Augment for Disturbances
Bd = [0 0 0 1/r0]';
Baug_OL = [B,Bd];
Dd = [0,0]';
Daug_OL = [D,Dd];

% OL Augmented System
OLsys = ss(A,Baug_OL,C,Daug_OL);

%% Full Input Profiles
rt1 = [rt(:),zeros(length(tspan),1),dt(:)]; % First thruster only
rt2 = [zeros(length(tspan),1),rt(:),dt(:)]; % Second thruster only

%% Simulate OL Dynamics
[y_OL1,~,x_OL1] = lsim(OLsys,rt1,tspan,delta_x0);
[y_OL2,~,x_OL2] = lsim(OLsys,rt2,tspan,delta_x0);

%% Determine Reachability and Observability (and Their Subspaces) of 
% Determine Reachability and Observability
P = ctrb(A,Baug_OL);
rank(P)
Ob = obsv(A,C);
rank(Ob)

% Determine Subspaces
rangeP = orth(P);
rangeOb = orth(Ob);

%% Augment for Integral Control
Aaug_OL = [A zeros(4,2);
            -C zeros(2,2)];
Baug_OL2 = [Baug_OL;
            zeros(2,3)];

Caug_OL = [C zeros(2,2)];

P_OL2 = ctrb(Aaug_OL,Baug_OL2);
rank(P_OL2)

% Unity Feedforward Gain
Faug = [zeros(size(Baug_OL));
         [eye(2),[0;0]]];

% Desired Pole Locations and Feedback Gain
des_poles_aug = [-1 -0.9 -0.8 -0.7 -0.6 -0.3];
Kaug = place(Aaug_OL,Baug_OL2,des_poles_aug);

% CL Augmented SS Matrices
Aaug_CL = Aaug_OL-Baug_OL2*Kaug;
Baug_CL = Faug;
Caug_CL = Caug_OL;
Daug_CL = Daug_OL;
augCL_sys = ss(Aaug_CL,Baug_CL,Caug_CL,Daug_CL);

% Simulate Augmented CL Dynamics
[yaug_CL1,~,xaug_CL1] = lsim(augCL_sys,rt1,tspan,delta_x0_int_control);
[yaug_CL2,~,xaug_CL2] = lsim(augCL_sys,rt2,tspan,delta_x0_int_control);

% Calculate Input u(t)
u1_aug = -Kaug*xaug_CL1';
u2_aug = -Kaug*xaug_CL2';
u1_aug = u1_aug';
u2_aug = u2_aug';

%% Plots
% System Responses for OL System
figure();
subplot(221)
plot(tspan,x_OL1(:,1),'LineWidth',2)
grid on; grid minor
xlabel('Time [s]')
ylabel('$\delta r$ [km]','Interpreter','latex')
title('x_1(t) Response for r_1(t)')
subplot(223)
plot(tspan,x_OL1(:,2),'LineWidth',2)
grid on; grid minor
xlabel('Time [s]')
ylabel('$\delta \dot{r}$ [km/s]','Interpreter','latex')
title('x_2(t) Response for r_1(t)')
subplot(222)
plot(tspan,x_OL1(:,3),'LineWidth',2)
grid on; grid minor
xlabel('Time [s]')
ylabel('$\delta \theta$ [rad]','Interpreter','latex')
title('x_3(t) Response for r_1(t)')
subplot(224)
plot(tspan,x_OL1(:,4),'LineWidth',2)
grid on; grid minor
xlabel('Time [s]')
ylabel('$\delta \dot{\theta}$ [rad/s]','Interpreter','latex')
title('x_4(t) Response for r_1(t)')
sgtitle('Open Loop Response for r_1(t)')

figure();
subplot(221)
plot(tspan,x_OL2(:,1),'LineWidth',2)
grid on; grid minor
xlabel('Time [s]')
ylabel('$\delta r$ [km]','Interpreter','latex')
title('x_1(t) Response for r_2(t)')
subplot(223)
plot(tspan,x_OL2(:,2),'LineWidth',2)
grid on; grid minor
xlabel('Time [s]')
ylabel('$\delta \dot{r}$ [km/s]','Interpreter','latex')
title('x_2(t) Response for r_2(t)')
subplot(222)
plot(tspan,x_OL2(:,3),'LineWidth',2)
grid on; grid minor
xlabel('Time [s]')
ylabel('$\delta \theta$ [rad]','Interpreter','latex')
title('x_3(t) Response for r_2(t)')
subplot(224)
plot(tspan,x_OL2(:,4),'LineWidth',2)
grid on; grid minor
xlabel('Time [s]')
ylabel('$\delta \dot{\theta}$ [rad/s]','Interpreter','latex')
title('x_4(t) Response for r_2(t)')
sgtitle('Open Loop Response for r_2(t)')

% Output Responses CL Integral Control
figure();
subplot(221)
plot(tspan,yaug_CL1(:,1),'LineWidth',2)
grid on; grid minor
xlabel('Time [s]')
ylabel('$\delta r$ [km]','Interpreter','latex')
title('y_1(t) Response for r_1(t)')
subplot(223)
plot(tspan,yaug_CL1(:,2),'LineWidth',2)
grid on; grid minor
xlabel('Time [s]')
ylabel('$\delta \dot{r}$ [rad]','Interpreter','latex')
title('y_2(t) Response for r_1(t)')
subplot(222)
plot(tspan,yaug_CL2(:,2),'LineWidth',2)
grid on; grid minor
xlabel('Time [s]')
ylabel('$\delta r$ [km/s]','Interpreter','latex')
title('y_1(t) Response for r_2(t)')
subplot(224)
plot(tspan,yaug_CL2(:,2),'LineWidth',2)
grid on; grid minor
xlabel('Time [s]')
ylabel('$\delta \dot{\theta}$ [rad/s]','Interpreter','latex')
title('y_2(t) Response for r_2(t)')

% Actuator Responses CL Integral Control
figure();
subplot(121)
plot(tspan,u1_aug(:,1),'LineWidth',2)
hold on; grid on; grid minor
plot(tspan,u1_aug(:,2),'LineWidth',2)
plot(tspan,u_max*ones(length(tspan)),'k:','LineWidth',2)
plot(tspan,-u_max*ones(length(tspan)),'k:','LineWidth',2)
xlabel('Time [s]')
ylabel('Radial Thruster \delta u_{1,2} [km/s^2]')
title('Thruster Response to r_1(t)')
subplot(122)
plot(tspan,u2_aug(:,1),'LineWidth',2)
hold on; grid on; grid minor
plot(tspan,u2_aug(:,2),'LineWidth',2)
plot(tspan,u_max*ones(length(tspan)),'k:','LineWidth',2)
plot(tspan,-u_max*ones(length(tspan)),'k:','LineWidth',2)
xlabel('Time [s]')
ylabel('Thruster Input \delta u_{1,2} [km/s^2]')
title('Thruster Response to r_2(t)')
legend('u_1(t)','u_2(t)','u_{max}(t) = \pm\pi/2')

