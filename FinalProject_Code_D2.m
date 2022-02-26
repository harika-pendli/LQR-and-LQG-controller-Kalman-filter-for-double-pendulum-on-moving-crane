%% ========================================================================
%                         MATLAB Code - Part D2
%                    LQR controller - Non Linearized System
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

% State Space Representation of the system
A= [0 1 0 0 0 0; 0 0 (-g*m1)/M 0 (-g*m2)/M 0; 0 0 0 1 0 0;
    0 0 (-g*(m1+M))/(M*l1) 0 (-g*m2)/(M*l1) 0; 0 0 0 0 0 1;
    0 0 (-g*m1)/(M*l2) 0 (-g*(m2+M))/(M*l2) 0];

B= [0; 1/M; 0;
    1/(M*l1); 0; 1/(M*l2)];

C = [1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0];

D = 0; 

% Matrix Rank calculator
matrix_rank = rank([B A*B (A^2)*B (A^3)*B (A^4)*B (A^5)*B]);
disp("rank of matrix is " + matrix_rank)

% If loop to check if matrix is controllable
if matrix_rank == 6
disp('This System is Controllable')
else
disp('This System is Not Controllable')
end

%% ========================================================================
%                                LQR Design
% =========================================================================


Q = diag([7000 0 700000 0 7000000 70000000]);
R = 0.25;

% State FeedBack 'K'
K = lqr(A,B,Q,R);
disp(K)

% X= (A-B*K)x + B*U
A_N=[A-B*K];
States={'x' 'x_dot' 'th1' 'th1_dot' 'th2' 'th2_dot'};
Input={'F'};
Outputs = {'x'; 'alpha1'; 'alpha2'};

% Converting to a State Space Model
sys=ss(A_N,B,C,0,'statename',States,'inputname',Input,'outputname',Outputs);

% Initial Value Conditions:
X=[5,0,30*pi/180,0,45*pi/180,0];
t = 0:0.01:200;
dim_t= size(t);
F = zeros(dim_t);
[Y, t_T, X_T] = lsim(sys, F, t, X);
size(Y)

% Plot Plotting
figure,
plot(t, Y(:,1),'blue');
title('Crane Position over time') % displacement in x-direction only
xlabel('Time (s)'); 
ylabel('Crane Position');
figure,
plot(t, Y(:,2),'r',t,Y(:,3),'b');
title('Pendulum angle change over time')
xlabel('Time (s)'); 
ylabel('Theta value of Pendulum ');
legend('pendulum 1', 'pendulum 2')

%% ========================================================================
%                         Lyapunov Indirect Method
% =========================================================================

Ly_St= eig(A_N)
if real(Ly_St)<1
disp('This System is Stable')
else
disp('This System is Unstable')
end

