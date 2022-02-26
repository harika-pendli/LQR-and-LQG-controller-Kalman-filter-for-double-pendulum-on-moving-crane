%% ========================================================================
%                         MATLAB Code - Part E
%                        Checking Observability
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

A_dim = 6;
% Checks for Different Cases


% Case 1: x(t)
C1=[1 0 0 0 0 0];
Rank_1 = rank([C1;C1*A;C1*(A^2);C1*(A^3);C1*(A^4);C1*(A^5)])
disp('Case 1 -->')
if Rank_1 == A_dim
disp('This System is Observable')
else
disp('This System is not Observable')
end

% Case 2: (th1,th2)
C2=[0 0 1 0 0 0; 0 0 0 0 1 0];
Rank_2 = rank([C2;C2*A;C2*(A^2);C2*(A^3);C2*(A^4);C2*(A^5)])
disp('Case 2 -->')
if Rank_2 == A_dim
disp('This System is Observable')
else
disp('This System is not Observable')
end

% Case 3: (x,th2)
C3=[1 0 0 0 0 0
0 0 0 0 1 0];
Rank_3 = rank([C3;C3*A;C3*(A^2);C3*(A^3);C3*(A^4);C3*(A^5)])
disp('Case 3 -->')
if Rank_3 == A_dim
disp('This System is Observable')
else
disp('This System is not Observable')
end
% Case 4: (x,th1,th2)
C4=[1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];

Rank_4 = rank([C4;C4*A;C4*(A^2);C4*(A^3);C4*(A^4);C4*(A^5)])
disp('Case 4 -->')
if Rank_4 == A_dim
disp('This System is Observable')
else
disp('This System is not Observable')
end