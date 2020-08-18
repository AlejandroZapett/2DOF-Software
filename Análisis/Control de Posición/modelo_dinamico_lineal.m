clc;
clear all;
close all;

% Parámetros
theta = load('../theta.mat').theta;
%theta = load('../theta4.mat').theta;

SampleTime = 0.01;

% Representación matricial
syms u1 u2 q1 q2 dq1 dq2;

M11 = (theta(1) + theta(2)*cos(q2));
M12 = (theta(3) + theta(4)*cos(q2));
M21 = (theta(11) + theta(12)*cos(q2));
M22 = theta(13);

C11 = theta(5)*sin(q2)*dq2;
C12 = theta(6)*sin(q2)*dq2;
C21 = theta(14)*sin(q2)*dq1;
C22 = 0;

G1 = theta(7)*cos(q1)+theta(8)*cos(q1+q2);
G2 = theta(14)*cos(q1+q2);

% NOTA:Para obtener una aproximación lineal del sistema,
% no se puede utilizar la fricción de Coulomb.

% F1 = theta(9)*dq1 + theta(10)*sign(dq1);
% F2 = theta(16)*dq2 + theta(17)*sign(dq2);
F1 = theta(9)*dq1;
F2 = theta(16)*dq2;

M = [M11 M12;M21 M22];
C = [C11 C12;C21 C22];
G = [G1;G2];
F = [F1;F2];
T = [u1;u2];

% Modelo dinámico lineal
syms D(q1,q2,dq1,dq2,u1,u2);

D(q1,q2,dq1,dq2,u1,u2) = -inv(M)*(C*[dq1;dq2]+F-T);

A = [0*eye(2)                                           ...
    eye(2);                                             ...
    jacobian(D(q1,q2,dq1,dq2,u1,u2),[q1,q2])        ...
    jacobian(D(q1,q2,dq1,dq2,u1,u2),[dq1,dq2])];

B = [0*eye(2);                                          ...
    jacobian(D(q1,q2,dq1,dq2,u1,u2),[u1,u2])]; 

% Punto de equilibrio
q1 = 90*pi/180;
q2 = 0*pi/180;
dq1 = 0;
dq2 = 0;

G = subs(G)';

u1 = G(1,1);
u2 = G(1,2);

Al = subs(A);
Bl = subs(B);

A=zeros(4,4);
B=zeros(4,2);

for i=1:1:4
    for j=1:1:4
        A(i,j) = Al(i,j);
    end
end

for i=1:1:4
    for j=1:1:2
        B(i,j) = Bl(i,j);
    end
end

% Matriz de transferencia Ts
C = eye(4);
D = zeros(4,2);

sys = ss(A,B,C,D);
Ts = tf(sys); % Matriz de funciones de transferencia

