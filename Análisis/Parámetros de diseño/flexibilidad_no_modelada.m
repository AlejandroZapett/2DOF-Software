% Propiedades de los eslabones
clc;
clear all;
close all;

l1 = 0.012; % metros
lc1 = 0.006;
l2 = 0.013;
lc2 = 0.0065;

b = 0.07;
h = 0.04;
r = 0.011
e = 0.002;

rho = 0.9;
E = 2e+09;

V1 = l1*b*h-l1*(b-e)*(h-e);
V2 = l2*pi*r^2-l2*pi*(r-e)^2;

m2 = 0.172;
me1 = (rho)*V1;
me2 = (rho)*V2;

I = (1/12)*b*h^3-(1/12)*(b-e)*(h-e)^3;

g = 9.78;

y_max = -(g*l1^2*(3*l1*me1+4*m2*l1+12*lc2*me2))/(24*E*I)

x = 0:0.0001:l1;
y = deflexion(x);
plot(x,y);
xlabel('x [m]');
ylabel('y [m]');
title('Deflexión en el eslabón 1');