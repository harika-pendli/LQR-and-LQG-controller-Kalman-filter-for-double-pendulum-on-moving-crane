%% ========================================================================
%                         MATLAB Code - Part F
%                            LQG Controller
% =========================================================================
clc
clear

%% ========================================================================% 

% variables that are known
M = 1000; % mass of crane (kg)
m1 = 100; % mass of pendulum 1 (kg)
m2 = 100; % mass of pendulum 2 (kg)
l1 = 20; % cable length of pendulum 1  (m)
l2 = 10; % cable length of pendulum 2  (m)
g = 9.81; % gravity m/s^2

% State Space Representation of the system
A= [0 1 0 0 0 0; 0 0 (-g*m1)/M 0 (-g*m2)/M 0; 0 0 0 1 0 0;
    0 0 (-g*(m1+M))/(M*l1) 0 (-g*m2)/(M*l1) 0; 0 0 0 0 0 1;
    0 0 (-g*m1)/(M*l2) 0 (-g*(m2+M))/(M*l2) 0];

B= [0; 1/M; 0;
    1/(M*l1); 0; 1/(M*l2)];

C = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];

% Considering FeedBack Control for the Case x(t)
C1 = [1 0 0 0 0 0]; 
Q = diag([7000 0 700000 0 7000000 70000000]);
R = 0.25;
disp('State FeedBack:')

% State FeedBack 'K'
K = lqr(A,B,Q,R);
disp(K)
Poles = [-1 -2 -3 -4 -5 -6];   % arbitarily chosen
L1 = place(A',C1',Poles)';

% Setting up Kalman Estimator 
state_space = ss(A,[B B],C,0);
sensor = [1]; W = [1]; R1 = 0.25; Q1 =0.25;
[~,L,~] = kalman(state_space,Q1,R1,[],sensor,W);

% State Parameters for Plots:
States = {'x' 'x_dot' 'th1' 'th1_dot' 'the2' 'th2_dot','e1','e2','e3','e4','e5','e6'};
input = {'F'};
Outputs = {'x'};
A_L1 = [(A-B*K) (B*K); zeros(6,6) (A-L1*C1)];
B_L1 = [B ; zeros(size(B))];
C_L1 = [C1 zeros(size(C1))];
stat_space2 = ss(A_L1,B_L1,C_L1,0,'statename',States,'inputname',input,'outputname',Outputs);
X = [10;0;90*pi/180;0;45*pi/180;0;0;0;0;0;0;0];     % estimating all states to be zero
t = 0:0.01:200;
dim_t= size(t);
F = zeros(dim_t);
[Y, t_T, X_T] = lsim(stat_space2, F, t, X);

% Plot Plotting
figure(1)
plot(t,Y(:,1))
ylabel('Crane Position (m)')
xlabel('Time(s)')
title('Crane position over Time')

% figure(2)
% plot(t, Y(:,2),'r',t,Y(:,3),'b');
% title('Pendulum angle change over time')
% xlabel('Time (s)'); 
% ylabel('Theta value of Pendulum (rad)');
% legend('pendulum 1', 'pendulum 2')

