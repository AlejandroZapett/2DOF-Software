modelo_dinamico_lineal;

acelerometro = tf([1],[6.25e-04 1]);
encoder = tf([1],[5e-05 1]);

C1 = pid(12,0,1);
C2 = pid(0.5,0,0.1);

sys1_Q1 = feedback(C1*Ts(1,1),acelerometro);
sys2_Q1 = feedback(C1*Ts(1,1),encoder);
sys3_Q1 = feedback(C1*Ts(1,1),1);

sys1_Q2 = feedback(C2*Ts(2,2),acelerometro);
sys2_Q2 = feedback(C2*Ts(2,2),encoder);
sys3_Q2 = feedback(C2*Ts(2,2),1);

subplot(3,1,1);
step(encoder);hold on;
step(acelerometro);
legend('encoder','acelerómetro');

subplot(3,1,2);
step(sys1_Q1); hold on;
step(sys2_Q1); hold on;
legend('retroalimentación con encoder','retroalimentación con acelerómetro');

subplot(3,1,3);
step(sys1_Q2); hold on;
step(sys2_Q2); hold on;
legend('retroalimentación con encoder','retroalimentación con acelerómetro');