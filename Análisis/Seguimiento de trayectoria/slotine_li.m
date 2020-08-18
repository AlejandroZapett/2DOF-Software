%% Control Basado en Pasividad Estandar (Slotine Li)

clc;
clear all;
close all;

% Parámetros
theta = load('../theta.mat').theta;

% Ganancias de control
K = [20 0;  ...
     0 20];
  
L = [15 0;  ...
     0 15];
  
% Condiciones iniciales
q0 = [90*pi/180;0;0;0];

%% Gráficas y simulaciones
close all;

tmax = 40;

sim('sim_passivity_bassed_control',[0 tmax]);

t = ans.t;
q = ans.q;
u = ans.u;
q1d = ans.q1d;
q2d = ans.q2d;
dq1d = ans.dq1d;
dq2d = ans.dq2d;
xp = ans.xp;
yp = ans.yp;
xpd = ans.xpd;
ypd = ans.ypd;

figure;

% Q1
subplot(3,2,1);
plot(t,q(:,1),'b');hold on;
plot(t,q1d,'r');
xlabel('tiempo [s]');
ylabel('q1 [rad]');

% Q2
subplot(3,2,2);
plot(t,q(:,2),'b');hold on;
plot(t,q2d,'r');
xlabel('tiempo [s]');
ylabel('q2 [rad]');

% DQ1
subplot(3,2,3);
plot(t,q(:,3),'b');hold on;
plot(t,dq1d,'r');
xlabel('tiempo [s]');
ylabel('dq1 [rad]');

% DQ2
subplot(3,2,4);
plot(t,q(:,4),'b');hold on;
plot(t,dq2d,'r');
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

figure;
plot(xp,yp); hold on;
plot(xpd,ypd);
xlim([0 .12]);
ylim([0.19 0.26]);
