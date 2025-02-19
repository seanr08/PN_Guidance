%% MISSILE AUTOPILOT 1: PN Guidance
clc;clear;
% SI Unit conv.

d2r=pi/180;

% TARGET INIT COND

V_T=600;
X_T0=100;
Z_T0=20000;
l_T0=-2.5*d2r;

% MISSILE

X_M0=2000;
Z_M0=0;
l_M0=80*d2r;

% CONTROL LOOP GAINS

Kdc=1.12;
Ka=4.5;
Ki=14.3;
Kr=-0.37;