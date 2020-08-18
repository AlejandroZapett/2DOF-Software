clc;
clear all;
close all;

% %% Parámetros %% %
%TODO: Actualizar valores
% Dimensiones de los eslabones
l1 = 0.12;
l2 = 0.14;
b = 0.07;
h = 0.04;
e = 0.002;

rho = 900;

V1 = l1*b*h-l1*(b-e)*(h-e);
V2 = l2*b*h-l1*(b-e)*(h-e);

me1 = (rho)*V1; % Valor estimados eslabón 1
me2 = (rho)*V2; % Valor estimados eslabón 2
% Masas de los eslabones
me = 0.081; % Valor experimental encoder
mm = 0.091; % Valor experimental motor
m1 = me + mm + me1; % masa del motor 1, del encoder 1, y del eslabón 1
m2 = me + mm + me2; % masa del motor 2, del encoder 2, y del eslabón 2

% Centros de masa
lc1 = ( me1*(l1/2) )/(me1+mm+me);
lc2 = ( me2*(l2/2) )/(me1+mm+me);

% Dimensiones del motor y del sensor
rm = 0.025;
rs = 0.045;

% Inercias rotacionales
Izz1 = (me1/12)*(l1^2+h^2) + (mm/2)*rm^2 + (me/2)*rs^2 + m1*lc1^2; % 
Izz2 = (me2/12)*(l2^2+h^2) + (mm/2)*rm^2 + (me/2)*rs^2 + m2*lc2^2; % 

g = 9.78;

% Se decide prescindir de la presencia de la fricción de Coulomb debido
% a que genera ec. diferenciales no lineal y discontinuas.
% El resolvedor ODE45 no es capaz de lidear con este tipo de funciones.
% b = 0.1;

% Modelo dinámico 
syms q1(t) q2(t) u1 u2;

u1 = (Izz1+Izz2+(lc1^2)*m1+(l1^2+lc2^2)*m2+2*l1*lc2*m2*cos(q1(t)))*diff(diff(q1(t)))     ...
    +(Izz1+(lc2^2)*m2+l1*lc2*m2*cos(q2(t)))*diff(diff(q2(t)))                              ...
    -2*l1*lc2*m2*sin(q2(t))*diff(q1(t))*diff(q2(t))-l1*lc2*m2*sin(q2(t))*(diff(q2(t))^2) ...
    +lc1*m1*g*cos(q1(t))+l1*m2*g*cos(q1(t))+lc2*m2*g*cos(q1(t)+q2(t));

u2 = (Izz1+(lc2^2)*m2+l1*lc2*m2*cos(q2(t)))*diff(q1(t),t,t)             ...
    +(Izz2+(lc2^2)*m2)*diff(q2(t),t,t)                                  ...
    +l1*lc2*m2*sin(q2(t))*(diff(q1(t))^2)+g*lc2*m2*cos(q1(t)+q2(t));

% Representación matricial
syms tau1 tau2 q1 q2 dq1 dq2;

M11 = (Izz1+Izz2+(lc1^2)*m1+(l1^2+lc2^2)*m2+2*l1*lc2*m2*cos(q1));
M12 = (Izz1+(lc2^2)*m2+l1*lc2*m2*cos(q2));
M21 = M12;
M22 = (Izz2+(lc2^2)*m2);

C11 = -2*l1*lc2*m2*sin(q2)*dq2;
C12 = -l1*lc2*m2*sin(q2)*dq2;
C21 = l1*lc2*m2*sin(q2)*dq1;
C22 = 0;

G1 = lc1*m1*g*cos(q1)+l1*m2*g*cos(q1)+lc2*m2*g*cos(q1+q2);
G2 = g*lc2*m2*cos(q1+q2);

% F1 = b*dq1 + c*sign(dq1);
% F2 = b*dq2 + c*sign(dq2);
% F1 = b*dq1;
% F2 = b*dq2;

M = [M11 M12;M21 M22];
C = [C11 C12;C21 C22];
G = [G1;G2];
%F = [F1;F2];
T = [tau1;tau2];

% Modelo dinámico lineal
syms D(q1,q2,dq1,dq2,tau1,tau2);

D(q1,q2,dq1,dq2,tau1,tau2) = -inv(M)*(C*[dq1;dq2]+G-T);

A = [0*eye(2)                                           ...
    eye(2);                                             ...
    jacobian(D(q1,q2,dq1,dq2,tau1,tau2),[q1,q2])        ...
    jacobian(D(q1,q2,dq1,dq2,tau1,tau2),[dq1,dq2])];

B = [0*eye(2);                                          ...
    jacobian(D(q1,q2,dq1,dq2,tau1,tau2),[tau1,tau2])]; 

% Punto de equilibrio
q1 = 60*pi/180;
q2 = -30*pi/180;
dq1 = 0;
dq2 = 0;

tau1 = subs(u1);
tau2 = subs(u2);

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
Ts = tf(sys);

[numq1,denq1] = tfdata(Ts(1,1),'v');
[numq2,denq2] = tfdata(Ts(2,2),'v');


