function  [r,theta] =mincuadm(y,fi,Nob,p,n)

theta=[1:p]'; %vector columna de parámetros  

%condición inicial del vector de parámetros

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
c = 0.1;

t1m = [(Izz1+Izz2+(lc1^2)*m1+(l1^2+lc2^2)*m2),   ...
   2*l1*lc2*m2,                                  ...
   (Izz1+(lc2^2)*m2),                            ...
   l1*lc2*m2];

tc1 = [2*l1*lc2*m2 l1*lc2*m2];

tg1 = [g*(lc1*m1+l1*m2) g*lc2*m2];


tf1 = [b c];

t2m = [(Izz2+(lc2^2)*m2) l1*lc2*m2      ...
    (Izz2+(lc2^2)*m2)];

tc2 = [l1*lc1*m2];

tg2 = [g*lc2*m2];

tf2 = tf1;


theta = 1000*[t1m tc1 tg1 tf1 t2m tc2 tg2 tf2]';

psi=zeros(p,n); %vector columna de observaciones
P=eye(p,p);%10e3; %matriz de covarianza P
I=eye(n,n); %matriz identidad
ys=zeros(n,1);

  for k=1:Nob
    for j=1:n
        for i=1:p %se forma el regresor
            psi(i,j)=fi(k+Nob*(j-1),i);
        end
        for i=1:n 
            ys(i,1)=y(k+Nob*(i-1));    
        end
    end
    
    for o=1:p
        r(k,o) = theta(o);
    end
    
    e=ys-psi'*theta; %error de regresión
    theta= theta+P*psi*(I+psi'*P*psi)^(-1)*e;%vector estimado
    P=P-(P*psi*(I+psi'*P*psi)^(-1)*(psi')*P);%Matriz de covarianza
  end
end