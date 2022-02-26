%% ========================================================================
%                         MATLAB Code - Part D1
%                    to design the LQR controller 
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
% X = A*x +B*u
A= [0 1 0 0 0 0; 0 0 (-g*m1)/M 0 (-g*m2)/M 0; 0 0 0 1 0 0;
    0 0 (-g*(m1+M))/(M*l1) 0 (-g*m2)/(M*l1) 0; 0 0 0 0 0 1;
    0 0 (-g*m1)/(M*l2) 0 (-g*(m2+M))/(M*l2) 0];

B= [0; 1/M; 0;
    1/(M*l1); 0; 1/(M*l2)];

C = [1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0];

D = 0; 

% Creating Q matrix
Q = (C')*(C);
Q(1,1) = 7000;
Q(2,2) = 0;
Q(3,3) = 700000;
Q(4,5) = 0;
Q(5,5) = 7000000;
Q(6,6) = 70000000;

R=0.25; % Initializing R 

%% ========================================================================
%                                LQR Design
% =========================================================================

K = lqr(A, B, Q, R); % Optimal Gain Matrix K
A_new = (A - (B*K)); % new A value

% Observability Matrix
State = {'x' 'x_dot' 'th1' 'th1_dot' 'th2' 'th2_dot'};
Inputs = {'r'};
Outputs = {'x'; 'beta1'; 'beta2'};

% State Space Model
ClosSS = ss(A_new, B, C, D,'statename', State, 'inputname', Inputs, 'outputname', Outputs);

% Initializing Conditions
X0 = [5,0,30*pi/180,0,45*pi/180,0];
t = 0:0.01:200; % running for 200 seconds
Temp = size(t);
F = zeros(Temp);

% Time Response of Dynamic System
[Y, tTemp, XTemp] = lsim(ClosSS, F, t, X0);

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
ylabel('Theta value of Pendulum ');
legend('pendulum 1', 'pendulum 2')
