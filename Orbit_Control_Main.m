clc; clear; close all

%% OL SS System
opengl('save','software')
% Constants
mu = 3.986e5; % km^3/s^2
r0 = 6678; % km

% ICs
% delta_x0 = [0 0 0 0]';
delta_x0 = [0.01 1e-6 5e-6 1e-9]';
delta_x0_int_cont = [0.01 1e-6 5e-6 1e-9 0 0]';

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
T = (2*pi)/sqrt(mu/r0^3);
delta_t = 10;
tspan = 0:delta_t:2*T;

% Max Acceleration from Thruster
u_max = 1e-3 * 1e-3;

% Reference Tracking
rt_km = 0.2*sign(double(tspan > 5340));
rt_rad = 0.0001*sign(double(tspan > 5340));

%% Disturbance Profile
% Constant Bias (due to Drag?)
dmag = -1e-9;
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
delta_rt1 = [rt_km(:),zeros(length(tspan),1),dt(:)]; % First thruster only
delta_rt2 = [zeros(length(tspan),1),rt_rad(:),dt(:)]; % Second thruster only

%% Simulate OL Dynamics
[y_OL1,~,x_OL1] = lsim(OLsys,zeros(size(delta_rt1)),tspan,delta_x0);
[y_OL2,~,x_OL2] = lsim(OLsys,zeros(size(delta_rt2)),tspan,delta_x0);

%% Determine Reachability and Observability (and Their Subspaces)
% Determine Reachability and Observability
P = ctrb(A,B);
rank(P)
Ob = obsv(A,C);
rank(Ob)

% Determine Subspaces
rangeP = orth(P);
rangeOb = orth(Ob);

%% Manual Pole Placement Simulation
% % Desired Pole Locations and Feedback Gain
% des_poles = [-0.00277 -0.00276 -0.00275 -0.0018];
% K = place(A,B,des_poles);
% 
% % Feedforward Gain
% F = inv(C/(-A+B*K)*B);
% 
% % CL SS Dynamics
% A_CL = A-B*K;
% B_CL = [B*F,Bd];
% C_CL = C;
% D_CL = zeros(2,3);
% CLsys = ss(A_CL,B_CL,C_CL,D_CL);
% 
% % Simulate Augmented CL Dynamics
% [yaug_CL1,~,xaug_CL1] = lsim(CLsys,delta_rt1,tspan,delta_x0);
% [yaug_CL2,~,xaug_CL2] = lsim(CLsys,delta_rt2,tspan,delta_x0);
% 
% % Calculate Input u(t)
% u1_aug = -K*xaug_CL1'+F*delta_rt1(:,1:2)';
% u2_aug = -K*xaug_CL2'+F*delta_rt2(:,1:2)';
% u1_aug = u1_aug';
% u2_aug = u2_aug';

% Integral Control Augmentation
A_aug_OL = [A zeros(4,2);
            -C zeros(2,2)];
B_aug_OL = [B;
            zeros(2,2)];

C_aug_OL = [C zeros(2,2)];
D_aug_OL = zeros(2,2);

P_OL = ctrb(A_aug_OL,B_aug_OL);
O_OL = obsv(A_aug_OL,C_aug_OL);
rank(P_OL)


% Unity feedforward gain
F_aug = [zeros(size(B));
         eye(2)];

% Desired Pole Locations and Feedback Gain
des_poles_aug = [-0.00279 -0.00278 -0.00277 -0.00276 -0.00275 -0.0018];
K_aug = place(A_aug_OL,B_aug_OL,des_poles_aug);

% CL Augmented SS Matrices
A_aug_CL = A_aug_OL-B_aug_OL*K_aug;
B_aug_CL = [F_aug,[Bd;0;0]];
C_aug_CL = C_aug_OL;
D_aug_CL = [D_aug_OL,[0;0]];
aug_CL_sys = ss(A_aug_CL,B_aug_CL,C_aug_CL,D_aug_CL);

% Simulate Augmented CL Dynamics
[yaug_CL1,~,xaug_CL1] = lsim(aug_CL_sys,delta_rt1,tspan,delta_x0_int_cont);
[yaug_CL2,~,xaug_CL2] = lsim(aug_CL_sys,delta_rt2,tspan,delta_x0_int_cont);

% Calculate Input u(t)
u1_aug = -K_aug*xaug_CL1';
u2_aug = -K_aug*xaug_CL2';
u1_aug = u1_aug';
u2_aug = u2_aug';

%% Plots
% OL Ouput Reponse
Plot_Outputs(tspan,[y_OL1,y_OL2],[delta_rt1,delta_rt2],'Open Loop Response')

% Output Responses CL Integral Control
% Plot_Outputs(tspan,[yaug_CL1,yaug_CL2],[delta_rt1,delta_rt2],...
%     'Feedforward Input Conditioning Control Ouputs')
Plot_Outputs(tspan,[yaug_CL1,yaug_CL2],[delta_rt1,delta_rt2],...
    'Integral Control Ouputs')

% Actuator Responses CL Integral Control
% Plot_Thruster_Reponses(tspan,[u1_aug,u2_aug],u_max,...
%     'Feedforward Input Conditioning Thruster Response')
Plot_Thruster_Reponses(tspan,[u1_aug,u2_aug],u_max,...
    'Integral Control Thruster Response')

% States CL Integral Control
% Plot_States(tspan,xaug_CL1,'r_1(t)',...
%     'CL Feedforward Input Conditioning Control Response for r_1(t)')
% Plot_States(tspan,xaug_CL2,'r_2(t)',...
%     'CL Feedforward Input Conditioning Control Response for r_2(t)')
Plot_States(tspan,xaug_CL1,'r_1(t)','CL Integral Control Response for r_1(t)')
Plot_States(tspan,xaug_CL2,'r_2(t)','CL Integral Control Response for r_2(t)')


