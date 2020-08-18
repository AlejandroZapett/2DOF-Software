%% Datos de simulaciones

clc;
clear all;
close all;

global u1 u2;

% Solución numérica del sistema
dt = 0.001;
tf = 100;
ts = 0:dt:tf;
condiciones_iniciales = [0;0;0;0];
[t,x]=ode45('modelo_espacio_estados',ts,condiciones_iniciales);

[ren,col]=size(t);
mediciones = ren;

% Obtención numérica del estado derivado
dx = zeros(ren,4);

for i=1:1:(tf/dt)+1
    dx(i,:) = modelo_espacio_estados(t(i),x(i,:));
end

% Predicción phi
z=zeros(ren,1);
q1 = x(:,1);
q2 = x(:,2);
dq1 = x(:,3);
dq2 = x(:,4);
ddq1 = dx(:,3);
ddq2 = dx(:,4);

% Salida Y (pares de entrada)
u1 = 0.02*sin(0.2*t)+0.03*cos(0.1*t)+0.01*sin(0.001*t+0.1);
u2 = 0.01*cos(0.1*t)+0.02*sin(0.3*t)+0.01*sin(0.001*t+0.1);

y = [u1 u2];

%% Algoritmo de Estimación 1
% Consideraciones del modelo:
% - Fricción viscosa
% - Dinámica de los actuadores truncada

M1 = [ddq1 cos(q2).*ddq1 ddq2 cos(q2).*ddq2];
C1 = [-sin(q2).*dq1.*dq2 -sin(q2).*(dq2.^2)];
G1 = [cos(q1) cos(q1+q2)]; 
F1 = [dq1];

M2 = [ddq1 cos(q2).*ddq1 ddq2];
C2 = [sin(q2).*(dq1.^2)];
G2 = [cos(q1+q2)];
F2 = [dq2];

pred = [M1 C1 G1 F1 z z z z z z;    ...
    z z z z z z z z z M2 C2 G2 F2];

[rastro,theta] = mincuadm(y,pred,mediciones,15,2,1);
theta % --> estimación de parámetros

% verificación

M1 = theta(1)*ddq1+theta(2)*cos(q2).*ddq1 +theta(3)*ddq2 +theta(4)*cos(q2).*ddq2;
C1 = theta(5)*sin(q2).*dq1.*dq2 +theta(6)*sin(q2).*(dq2.^2);
G1 = theta(7)*cos(q1) +theta(8)*cos(q1+q2); 
F1 = theta(9)*dq1;

M2 = theta(10)*ddq1 +theta(11)*cos(q2).*ddq1 +theta(12)*ddq2;
C2 = theta(13)*sin(q2).*(dq1.^2);
G2 = theta(14)*cos(q1+q2);
F2 = theta(15)*dq2;

ren1 = M1+C1+G1+F1;
ren2 = M2+C2+G2+F2;

% % %% Parámetros %% %
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
b = 0.1;


t1m = [(Izz1+Izz2+(lc1^2)*m1+(l1^2+lc2^2)*m2),   ...
    2*l1*lc2*m2,                                  ...
    (Izz1+(lc2^2)*m2),                            ...
    l1*lc2*m2];

tc1 = [2*l1*lc2*m2 l1*lc2*m2];

tg1 = [g*(lc1*m1+l1*m2) g*lc2*m2];

tf1 = [b];

t2m = [(Izz2+(lc2^2)*m2) l1*lc2*m2      ...
    (Izz2+(lc2^2)*m2)];

tc2 = [l1*lc1*m2];

tg2 = [g*lc2*m2];

tf2 = tf1;
 
theta_real_sim = 1000*[t1m tc1 tg1 tf1 t2m tc2 tg2 tf2]';

plot(t,rastro);
xlabel('Tiempo [s]');
ylabel('theta');

figure;
subplot(2,1,1);
plot(t,u1,'b');
hold on;
plot(t,ren1,'--r');
legend('Entrada conocida', 'Predicción con parámetros estimados');
xlabel('Tiempo [s]');
ylabel('u1');

subplot(2,1,2);
plot(t,u2,'b');
hold on;
plot(t,ren2,'--r');
legend('Entrada conocida', 'Predicción con parámetros estimados');
xlabel('Tiempo [s]');
ylabel('u2');
