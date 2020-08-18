%% Control por Par calculado

clc;
clear all;
close all;

% Parámetros
theta = load('../theta.mat').theta;

% Sintonización 
polos = [-1,-2];

kp = (polos(1)*polos(2));
kv = -(polos(1)+polos(2));

% Ganancias de control
Kp = [kp 0;  ...
      0 kp];
  
Kv = [kv 0;  ...
      0 kv];
  
% Condiciones iniciales
q0 = [90*pi/180;0*pi/180;0;0];

%% Gráficas y simulaciones
close all;

tmax = 40;

sim('sim_par_calculado',[0 tmax]);

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
