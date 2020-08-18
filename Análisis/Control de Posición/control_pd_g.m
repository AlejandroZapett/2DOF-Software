%% Control PD con compensación de gravedad

clc; 
clear all;
close all;

% Parámetros
theta = load('../theta.mat').theta;

Td1 = 1/3;
Kp1 = 100;

Td2 = 1/1.5;
Kp2 = 80;

% Ganancias de control
Kp = [Kp1 0;  ...
      0   Kp2];
  
Kd = [Kp1*Td1 0;  ...
      0       Kp2*Td2];

% Condiciones iniciales
q0 = [90*pi/180;0;0;0];

% Objetivo de control
q_ = [110*pi/180;-30*pi/180;0;0];

%% Gráficas y simulaciones
close all;

tmax = 20;

%sim('sim_control_pd_g_2DOF',[0 tmax]);
sim('sim_dinamica_completa_pd',[0 tmax]);

t = ans.t;
q = ans.q;
u = ans.u;

figure;

% Q1
subplot(3,2,1);
plot(t,q(:,1),'b');hold on;
line([0 tmax],[q_(1) q_(1)],'color','red');
xlabel('tiempo [s]');
ylabel('q1 [rad]');

% Q2
subplot(3,2,2);
plot(t,q(:,2),'b');hold on;
line([0 tmax],[q_(2) q_(2)],'color','red');
xlabel('tiempo [s]');
ylabel('q2 [rad]');

% DQ1
subplot(3,2,3);
plot(t,q(:,3),'b');
xlabel('tiempo [s]');
ylabel('dq1 [rad]');

% DQ2
subplot(3,2,4);
plot(t,q(:,4),'b');
xlabel('tiempo [s]');
ylabel('dq2 [rad]');

% u1
subplot(3,2,5);
plot(t,u(:,1),'b');
xlabel('tiempo [s]');
ylabel('u1 [pwm]');

% u2
subplot(3,2,6);
plot(t,u(:,2),'b');
xlabel('tiempo [s]');
ylabel('u2 [pwm]');

%% Datos experimentales

close all;

SampleTime = 0.01;
%SampleTime = 0.005;

% Una sóla posición
Table = readtable('../Reportes/control_pos_2.csv');

% Entrada escalón
%Table = readtable('../Reportes/escalon_2.csv');

u1 = Table.u1;
u2 = Table.u2;
q1 = Table.q1; 
q2 = Table.q2;
dq1 = Table.dq1;
dq2 = Table.dq2;

q1d = Table.q1d; 
q2d = Table.q2d;

ddq1 = diff(q1,2)/SampleTime;
ddq2 = diff(q2,2)/SampleTime;

[rows,columns] = size(u1);
t = (0:SampleTime:rows*SampleTime-SampleTime)';

% Ajustando tamaño de los arreglos
[len,col] = size(ddq1);
mediciones = len-2;

u1 = u1(2:len,1);
u2 = u2(2:len,1);

q1 = q1(2:len,1);
q2 = q2(2:len,1);

dq1 = dq1(2:len,1);
dq2 = dq2(2:len,1);

ddq1 = ddq1(2:len,1);
ddq2 = ddq2(2:len,1);

q1d = q1d(2:len,1);
q2d = q2d(2:len,1);

t = t(2:len,1);

figure;

% Q1
subplot(3,2,1);
plot(t,q1,'b');hold on;
plot(t,q1d,'r');
xlabel('tiempo [s]');
ylabel('q1 [rad]');
%xlim([0 4.15]);
%ylim([1.55 1.82])

% Q2
subplot(3,2,2);
plot(t,q2,'b');hold on;
plot(t,q2d,'r');
xlabel('tiempo [s]');
ylabel('q2 [rad]');
%xlim([0 4.15]);

% DQ1
subplot(3,2,3);
plot(t,dq1,'b');
xlabel('tiempo [s]');
ylabel('dq1 [rad]');
%xlim([0 4.15]);

% DQ2
subplot(3,2,4);
plot(t,dq2,'b');
xlabel('tiempo [s]');
ylabel('dq2 [rad]');
%xlim([0 4.15]);

% u1
subplot(3,2,5);
plot(t,u1,'b');
xlabel('tiempo [s]');
ylabel('u1 [pwm]');
%xlim([0 4.15]);

% u2
subplot(3,2,6);
plot(t,u2,'b');
xlabel('tiempo [s]');
ylabel('u2 [pwm]');
%xlim([0 4.15]);