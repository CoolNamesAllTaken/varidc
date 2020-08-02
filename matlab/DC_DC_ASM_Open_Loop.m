clear all;
close all;

s = tf('s');

n = 3; % turns ratio (n:1 transformer)
C_o = 33e-6; % output capacitor value [uF]
L_o = 50e-3; % output inductor value [mH]
R = 100; % output load value [Ohms]
D = 0.3;

% G_oc = n*R/D * 1/(s^2*L_o*C_o/2 + s*(L_o*R^2+L_o)/(2*R) + 1); % TF from i_c to v_o
G_oc = n*R/D /(s^2*L_o*C_o/2 + s*(C_o*R^2+L_o)/(2*R) + 1) % TF from i_c to v_o

C = 0.75 * (1+0.00067*s)/s;

% margin(G_oc)

bode(C)
figure()
bode(G_oc)
sisotool(G_oc, C)