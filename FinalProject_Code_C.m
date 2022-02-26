%% ========================================================================
%                         MATLAB Code - Part C
%       to find the determinant expression and rank of matrix 
% =========================================================================
clc
clear

%% ========================================================================
% State Space representation of the system
syms g m1 m2 l1 l2 M  % declaring variables
A= [0 1 0 0 0 0; 0 0 (-g*m1)/M 0 (-g*m2)/M 0; 0 0 0 1 0 0;
    0 0 (-g*(m1+M))/(M*l1) 0 (-g*m2)/(M*l1) 0; 0 0 0 0 0 1;
    0 0 (-g*m1)/(M*l2) 0 (-g*(m2+M))/(M*l2) 0];

B= [0; 1/M; 0;
    1/(M*l1); 0; 1/(M*l2)];

C = [1 0 0 0 0 0;
    0 0 1 0 0 0;
    0 0 0 0 1 0];
%% ========================================================================
matrix = [B A*B (A^2)*B (A^3)*B (A^4)*B (A^5)*B];
matrix_rank = rank([B A*B (A^2)*B (A^3)*B (A^4)*B (A^5)*B]);
disp("rank of matrix is " + matrix_rank)
det(matrix)


