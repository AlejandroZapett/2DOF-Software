clear all;
clc;
close all;

syms t l1 q1d(t) q2d(t) l2;

J = [-l1*sin(q1d)-l2*sin(q1d+q2d), ...
    -l2*sin(q1d+q2d);             ...
    l1*cos(q1d)+l2*cos(q1d+q2d),   ...
    l2*cos(q1d+q2d)];

JI = inv(J)

DJI = diff(JI,t);

DJI(1)
