%% ========================================================================
%                         MATLAB Code - Part F
%                       'best' Lueberger Observer
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

% Error Dynamic (A-L*C)*e
Q = diag([7000 0 700000 0 7000000 70000000]);
R = 0.25;
disp('State FeedBack:')

% State FeedBack 'K'
K = lqr(A,B,Q,R);
disp(K)
disp('Eigenvalues of (A-B*K) are as follows')

%  Pole Placement 
Eig_Vals = eig(A-B*K);
disp(Eig_Vals)

% Arbitrary Pole Placement
Poles = [-1 -2 -3 -4 -5 -6];

% State Outputs for Observable systems
C1 = [1 0 0 0 0 0];
C3 = [1 0 0 0 0 0
      0 0 0 0 1 0];
C4 = [1 0 0 0 0 0
      0 0 1 0 0 0
      0 0 0 0 1 0];
L1 = place(A', C1',Poles)';
L2 = place(A', C3',Poles)';
L3 = place(A', C4',Poles)';
 
% Case 1
    A_L1 = [(A-B*K) (B*K); zeros(6,6) (A-L1*C1)];
    B_L1 = [B; zeros(size(B))];
    C_L1 = [C1 zeros(size(C1))];

% Case 2
    A_L2 = [(A-B*K) (B*K); zeros(6,6) (A-L2*C3)];
    B_L2 = [B; zeros(size(B))];
    C_L2 = [C3 zeros(size(C3))];

% Case 3
    A_L3 = [(A-B*K) (B*K); zeros(6,6) (A-L3*C4)];
    B_L3 = [B; zeros(size(B))];
    C_L3 = [C4 zeros(size(C4))];

% State Space representation of the Linear system

stsp1 = ss(A_L1,B_L1,C_L1,0);
stsp2 = ss(A_L2,B_L2,C_L2,0);
stsp3 = ss(A_L3,B_L3,C_L3,0);

% Plots From STEP function
figure(1)
step(stsp1)
xlabel('Time ')
ylabel('Position (m)')
title('observability for Case 1: x(t)')

figure(2)
step(stsp2)
xlabel('Time ')
ylabel('Position (m)')
title('observability for Case 2: (x(t), th2)')

figure(3)
step(stsp3)
xlabel('Time ')
ylabel('Position (m)')
title('observability for Case 3: (x(t), th2, th1)')


