%% Datos experimentales
clc;
clear all;
close all;

SampleTime = 0.01;

Table = readtable('../Reportes/2DOF-IdenParam.csv');

u1 = Table.u1;
u2 = Table.u2;
q1 = Table.q1 * (pi/180); % PASO A RADIANES
q2 = Table.q2 * (pi/180); % PASO A RADIANES

dq1 = Table.dq1 * (pi/180);
dq2 = Table.dq2 * (pi/180);

ddq1 = diff(q1,2)/SampleTime;
ddq2 = diff(q2,2)/SampleTime;

[rows,columns] = size(u1);
t = (0:SampleTime:rows*SampleTime-SampleTime)';

% Ajustando tamaño de los arreglos
[len,col] = size(ddq1);
mediciones = len;

u1 = u1(1:len,1);
u2 = u2(1:len,1);

q1 = q1(1:len,1);
q2 = q2(1:len,1);

dq1 = dq1(1:len,1);
dq2 = dq2(1:len,1);

ddq1 = dq1(1:len,1);
ddq2 = dq2(1:len,1);

t = t(1:len,1);
z=zeros(len,1);

y = [u1 u2];

%% Algoritmo de Estimación 2
% Consideraciones del modelo:
% - Fricción viscosa y de coulomb
% - Dinámica de los actuadores truncada

M1 = [ddq1 cos(q2).*ddq1 ddq2 cos(q2).*ddq2];
C1 = [sin(q2).*dq1.*dq2 sin(q2).*(dq2.^2)];
G1 = [cos(q1) cos(q1+q2)]; 
F1 = [dq1 sign(dq1)];

M2 = [ddq1 cos(q2).*ddq1 ddq2];
C2 = [sin(q2).*(dq1.^2)];
G2 = [cos(q1+q2)];
F2 = [dq2 sign(dq2)];

pred = [M1 C1 G1 F1 z z z z z z z;    ...
    z z z z z z z z z z M2 C2 G2 F2];

[rastro,theta] = mincuadm(y,pred,mediciones,17,2);
theta % --> estimación de parámetros

% verificación

M1 = theta(1)*ddq1+theta(2)*cos(q2).*ddq1 +theta(3)*ddq2 +theta(4)*cos(q2).*ddq2;
C1 = theta(5)*sin(q2).*dq1.*dq2 +theta(6)*sin(q2).*(dq2.^2);
G1 = theta(7)*cos(q1) +theta(8)*cos(q1+q2); 
F1 = theta(9)*dq1+theta(10)*sign(dq1);

M2 = theta(11)*ddq1 +theta(12)*cos(q2).*ddq1 +theta(13)*ddq2;
C2 = theta(14)*sin(q2).*(dq1.^2);
G2 = theta(15)*cos(q1+q2);
F2 = theta(16)*dq2+theta(17)*sign(dq2);

ren1 = M1+C1+G1+F1;
ren2 = M2+C2+G2+F2;

plot(t,rastro);
xlabel('Tiempo [s]');
ylabel('theta');

figure;
subplot(2,1,1);
plot(t,u1)
hold on;
plot(t,ren1);
xlabel('tiempo');
ylabel('u1 [PWM]');
legend('u1 conocida','u1 estimada');

subplot(2,1,2);
plot(t,u2)
hold on;
plot(t,ren2);
xlabel('tiempo');
ylabel('u2 [PWM]');
legend('u2 conocida','u2 estimada');