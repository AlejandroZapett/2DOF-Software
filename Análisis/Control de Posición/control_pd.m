%% Apoximación lineal 

modelo_dinamico_lineal;

% Q1

G_q1_u1 = Ts(1,1);
[num,den]=tfdata(Ts(1,1),'v');

Td1 = 1/3;
Kp1 = 100;
C1 = pid(Kp1,0,Kp1*Td1);

H1 = feedback(C1*Ts(1,1),1);

%[n,d] = tfdata(H,'v');
%roots(d)
%step(H); ylabel('q2 [rad]');

% Q2

G_q2_u2 = Ts(2,2);
[num,den]=tfdata(Ts(2,2),'v');

Td2 = 1/1.5;
Kp2 = 80;
C2 = pid(Kp2,0,Kp2*Td2);

H2 = feedback(C2*Ts(2,2),1);

%[n,d] = tfdata(H,'v');
%roots(d);
%step(H); ylabel('q2 [rad]');

%% Análisis en tiempo discreto

Ts = 0.01;
z = tf('z',Ts);

MAX_T = 10;

% %% EQUIVALENTE DE LAS PLANTAS

% Equivlente ZOH para Q1
GZ1_q1 = c2d(G_q1_u1,Ts,'ZOH');

% Equivalente triangular FOH para Q1
GZ2_q1 = c2d(G_q1_u1,Ts,'FOH');

% Equivlente ZOH para Q1
GZ1_q2 = c2d(G_q2_u2,Ts,'ZOH');

% Equivalente triangular FOH para Q1
GZ2_q2 = c2d(G_q2_u2,Ts,'FOH');

% %% EQUIVALENTE DE LOS CONTROLADORES

% *************** Para: Q1 **************************
kp = C1.Kp;
ki = C1.Ki;
kd = C1.Kd;
%1. Regla de adelanto
CZ1_q1 = kp + (kd/Ts)*(z-1) + ki*Ts*(z-1)^(-1);

Y1_1_q1 = (CZ1_q1*GZ1_q1)/(1+CZ1_q1*GZ1_q1); % ZOH
Y1_2_q1 = (CZ1_q1*GZ2_q1)/(1+CZ1_q1*GZ2_q1); % FOH

% figure;step(Y1_1_q1,H1,1);
% figure;step(Y1_2_q1,H1,1); % --> inestable

%2. Regla de retraso
CZ2_q1 = kp + (kd/Ts)*(1-z^(-1)) + ki*Ts*(1-z^(-1))^(-1);

Y2_1_q1 = (CZ2_q1*GZ1_q1)/(1+CZ2_q1*GZ1_q1); % ZOH
Y2_2_q1 = (CZ2_q1*GZ2_q1)/(1+CZ2_q1*GZ2_q1); % FOH

% figure;step(Y2_1_q1,H1,1); % --> inestable
% figure;step(Y2_2_q1,H1,1); % --> muchas oscilaciones

%3. Regla trapezoidal
CZ3_q1 = c2d(C1,Ts,'tustin');

Y3_1_q1 = (CZ3_q1*GZ1_q1)/(1+CZ3_q1*GZ1_q1); % ZOH
Y3_2_q1 = (CZ3_q1*GZ2_q1)/(1+CZ3_q1*GZ2_q1); % FOH

% figure;step(Y3_1_q1,H1,1); % --> inestable
% figure;step(Y3_2_q1,H1,1); % --> inestable 

% 4. Mapeo de polos y zeros
CZ4_q1 = c2d(C1,Ts,'matched')

Y4_1_q1 = (CZ4_q1*GZ1_q1)/(1+CZ4_q1*GZ1_q1); % ZOH
Y4_2_q1 = (CZ4_q1*GZ2_q1)/(1+CZ4_q1*GZ2_q1); % FOH

% figure;step(Y4_1_q1,H1,1); 
% figure;step(Y4_2_q1,H1,1); % --> inestable

% *************** Para: Q2 **************************
kp = C2.Kp;
ki = C2.Ki;
kd = C2.Kd;
%1. Regla de adelanto
CZ1_q2 = kp + (kd/Ts)*(z-1) + ki*Ts*(z-1)^(-1);

Y1_1_q2 = (CZ1_q2*GZ1_q2)/(1+CZ1_q2*GZ1_q2); % ZOH
Y1_2_q2 = (CZ1_q2*GZ2_q2)/(1+CZ1_q2*GZ2_q2); % FOH

% figure;step(Y1_1_q2,H2,1);
% figure;step(Y1_2_q2,H2,1); % --> inestable

%2. Regla de retraso
CZ2_q2 = kp + (kd/Ts)*(1-z^(-1)) + ki*Ts*(1-z^(-1))^(-1);

Y2_1_q2 = (CZ2_q2*GZ1_q2)/(1+CZ2_q2*GZ1_q2); % ZOH
Y2_2_q2 = (CZ2_q2*GZ2_q2)/(1+CZ2_q2*GZ2_q2); % FOH

% figure;step(Y2_1_q2,H2,1); % --> inestable
% figure;step(Y2_2_q2,H2,1); % --> muchas oscilaciones

%3. Regla trapezoidal
CZ3_q2 = c2d(C2,Ts,'tustin');

Y3_1_q2 = (CZ3_q2*GZ1_q2)/(1+CZ3_q2*GZ1_q2); % ZOH
Y3_2_q2 = (CZ3_q2*GZ2_q2)/(1+CZ3_q2*GZ2_q2); % FOH

% figure;step(Y3_1_q2,H2,1); % --> inestable
% figure;step(Y3_2_q2,H2,1); % --> inestable

% 4. Mapeo de polos y zeros
CZ4_q2 = c2d(C2,Ts,'matched')

Y4_1_q2 = (CZ4_q2*GZ1_q2)/(1+CZ4_q2*GZ1_q2); % ZOH
Y4_2_q2 = (CZ4_q2*GZ2_q2)/(1+CZ4_q2*GZ2_q2); % FOH

% figure;step(Y4_1_q2,H2,1); 
% figure;step(Y4_2_q2,H2,1); % --> inestable

%% Gráficas 1

% Q1
figure; step(Y1_1_q1,Y2_1_q1,Y3_1_q1,Y4_1_q1,H1,MAX_T);
legend('R. de adelanto','R. de atraso','R. trapezoidal','Mapeo de p. y c.','tiempo continuo');
title('Respuesta a escalón unitario con ZOH');
figure; bode(Y1_1_q1,Y2_1_q1,Y3_1_q1,Y4_1_q1,H1);
legend('R. de adelanto','R. de atraso','R. trapezoidal','Mapeo de p. y c.','tiempo continuo');
title('Bode con ZOH');

figure; step(Y1_2_q1,Y2_2_q1,Y3_2_q1,Y4_2_q1,H1,MAX_T);
legend('R. de adelanto','R. de atraso','R. trapezoidal','Mapeo de p. y c.','tiempo continuo');
title('Respuesta a escalón unitario con FOH');
figure; bode(Y1_2_q1,Y2_2_q1,Y3_2_q1,Y4_2_q1,H1);
legend('R. de adelanto','R. de atraso','R. trapezoidal','Mapeo de p. y c.','tiempo continuo');
title('Bode con FOH');

% Q2
figure; step(Y1_1_q2,Y2_1_q2,Y3_1_q2,Y4_1_q2,H1,MAX_T);
legend('R. de adelanto','R. de atraso','R. trapezoidal','Mapeo de p. y c.','tiempo continuo');
title('Respuesta a escalón unitario con ZOH');
figure; bode(Y1_1_q2,Y2_1_q2,Y3_1_q2,Y4_1_q2,H1);
legend('R. de adelanto','R. de atraso','R. trapezoidal','Mapeo de p. y c.','tiempo continuo');
title('Bode con ZOH');

figure; step(Y1_2_q2,Y2_2_q2,Y3_2_q2,Y4_2_q2,H2,MAX_T);
legend('R. de adelanto','R. de atraso','R. trapezoidal','Mapeo de p. y c.','tiempo continuo');
title('Respuesta a escalón unitario con FOH');
figure; bode(Y1_2_q2,Y2_2_q2,Y3_2_q2,Y4_2_q2,H2);
legend('R. de adelanto','R. de atraso','R. trapezoidal','Mapeo de p. y c.','tiempo continuo');
title('Bode con FOH');

%% Gráficas 2

% Q1
figure;
subplot(4,2,1);
step(Y1_1_q1,H1,MAX_T);
legend('R. de adelanto','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q1 [rad]');
title('R. de adelanto con ZOH');

subplot(4,2,2);
step(Y2_1_q1,H1,MAX_T);
legend('R. de retraso','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q1 [rad]');
title('R. de retraso con ZOH');

subplot(4,2,3);
step(Y3_1_q1,H1,MAX_T);
legend('R. de trapezoidal','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q1 [rad]');
title('R. trapezoidal con ZOH');

subplot(4,2,4);
step(Y4_1_q1,H1,MAX_T);
legend('Mapeo de p. y c','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q1 [rad]');
title('Mapeo de p. y c. con ZOH');

subplot(4,2,5);
step(Y1_2_q1,H1,MAX_T);
legend('R. de adelanto','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q1 [rad]');
title('R. de adelanto con FOH');

subplot(4,2,6);
step(Y2_2_q1,H1,MAX_T);
legend('R. de retraso','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q1 [rad]');
title('R. de retraso con FOH');

subplot(4,2,7);
step(Y3_2_q1,H1,MAX_T);
legend('R. trapezoidal','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q1 [rad]');
title('R. trapezoidal con FOH');

subplot(4,2,8);
step(Y4_2_q1,H1,MAX_T);
legend('Mapeo de p. y c.','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q1 [rad]');
title('Mapeo de p. y c. con FOH');

% Q2
figure;
subplot(4,2,1);
step(Y1_1_q2,H2,MAX_T);
legend('R. de adelanto','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q2 [rad]');
title('R. de adelanto con ZOH');

subplot(4,2,2);
step(Y2_1_q2,H2,MAX_T);
legend('R. de retraso','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q2 [rad]');
title('R. de retraso con ZOH');

subplot(4,2,3);
step(Y3_1_q2,H2,MAX_T);
legend('R. trapezoidal','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q2 [rad]');
title('R. trapezoidal con ZOH');

subplot(4,2,4);
step(Y4_1_q2,H2,MAX_T);
legend('Mapeo de p. y c.','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q2 [rad]');
title('Mapeo de p. y c. con ZOH');

subplot(4,2,5);
step(Y1_2_q2,H2,MAX_T);
legend('R. de adelanto','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q2 [rad]');
title('R. de adelanto con FOH');

subplot(4,2,6);
step(Y2_2_q2,H2,MAX_T);
legend('R. de retraso','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q2 [rad]');
title('R. de retraso con FOH');

subplot(4,2,7);
step(Y3_2_q2,H2,MAX_T);
legend('R. trapezoidal','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q2 [rad]');
title('R. trapezoidal con FOH');

subplot(4,2,8);
step(Y4_2_q2,H2,MAX_T);
legend('Mapeo de p. y c.','Tiempo continuo');
xlabel('Tiempo [s]');
ylabel('q2 [rad]');
title('Mapeo de p. y c. con FOH');

