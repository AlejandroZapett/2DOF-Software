function y=deflexion(x)

% Propiedades de los eslabones

l1 = 0.012; % metros
lc1 = 0.006;
l2 = 0.013;
lc2 = 0.0065;

b = 0.07;
h = 0.04;
r = 0.011;
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

M2 = lc2*me2*g;
w2 = m2*g;
we1 = me1*g;

y=-(1/(E*I))*( (M2/2)*x.^2 + ((we1+w2)/6)*x.^3 - (we1/(24*l1))*x.^4 );
