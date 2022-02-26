%% ========================================================================
%                         MATLAB Code - Part B
% =========================================================================
clc
clear

%% ========================================================================
% variables that are known
M = 1000; % mass of crane (kg)
m1 = 100; % mass of pendulum 1 (kg)
m2 = 100; % mass of pendulum 2 (kg)
l1 = 20; % cable length of pendulum 1  (m)
l2 = 10; % cable length of pendulum 2  (m)
g = 9.81; % gravity m/s^2

% State Space representation of the system
% A is a 6x6 matrix
% B is a 6x1 matrix
% C is a 3x6 matrix

A= [0 1 0 0 0 0; 0 0 (-g*m1)/M 0 (-g*m2)/M 0; 0 0 0 1 0 0;
    0 0 (-g*(m1+M))/(M*l1) 0 (-g*m2)/(M*l1) 0; 0 0 0 0 0 1;
    0 0 (-g*m1)/(M*l2) 0 (-g*(m2+M))/(M*l2) 0];

B= [0; 1/M; 0;
    1/(M*l1); 0; 1/(M*l2)];

C = [1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0];

% Initial Value Conditions set by us
X = [5,0,30*pi/180,0,45*pi/180,0];
t = 0:0.01:200; % running for 200 seconds
dim_t = size(t);
F = zeros(dim_t);

% State parameters 
State = {'x' 'x_dot' 'th1' 'th1_dot' 'th2' 'th2_dot'};
input = {'F'};
Outputs = {'x'; 'alpha1'; 'alpha2'};

% States
system = ss(A,B,C,0,'statename',State,'inputname',input,'outputname',Outputs);
[Y, t_T, X_T] = lsim(system, F, t, X);

% Plot Plotting
figure,
plot(t, Y(:,1),'blue');
title('Crane Position over time') % displacement in x-direction only
xlabel('Time (s)'); 
ylabel('Crane Position (m)');

figure,
plot(t, Y(:,2),'r',t,Y(:,3),'b');
title('Pendulum angle change over time')
xlabel('Time (s)'); 
ylabel('Theta value of Pendulum (rad)');
legend('pendulum 1', 'pendulum 2')

