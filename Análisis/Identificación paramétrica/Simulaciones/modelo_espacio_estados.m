function dx = modelo_espacio_estados(t,x)

global u1 u2
u1 = 0.02*sin(0.2*t)+0.03*cos(0.1*t)+0.01*sin(0.001*t+0.1);
u2 = 0.01*cos(0.1*t)+0.02*sin(0.3*t)+0.01*sin(0.001*t+0.1);

q1 = x(1);
q2 = x(2);
dq1 = x(3);
dq2 = x(4);

% %% Parámetros %% %
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

% %% Modelo dinámico %% %

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

% Se decide prescindir de la presencia de la fricción de Coulomb debido
% a que genera ec. diferenciales no lineal y discontinuas.
% El resolvedor ODE45 no es capaz de lidear con este tipo de funciones.
F1 = b*dq1;
F2 = b*dq2;

M = 1000*[M11 M12;M21 M22];
C = 1000*[C11 C12;C21 C22];
G = 1000*[G1;G2];
F = 1000*[F1;F2];

Tau = [u1;u2];

dx1 = dq1;
dx2 = dq2;
% D = -inv(R*M+J)*((R*C+B)*[dq1;dq2]+R*G+R*F-K*V); % Modelo D. Completo
D = -M\(C*[dq1;dq2]+G+F-Tau); % Modelo dinámico truncado

dx = [dx1;dx2;D];