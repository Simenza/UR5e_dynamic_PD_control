% ==========================================================
% Industrial Robotics
% Dynamic model of UR5e
% Adaptation of symbolic 2R exercise to 6-DOF robot
%
% Control developed in Simulink
% ==========================================================

clear all;
clc;


% Definizione del robot
L = [0.1625, 0.425, 0.3922, 0.1333, 0.0997, 0.0996];

% Masses
m = [3.5, 8, 2.5, 1.5, 0.5, 0.5];

% Centri di massa locali (x, y, z) rispetto al frame link
r = [ 0 0 L(1)/2;       % link1
      0 0 L(2)/2;       % link2
      0 0 L(3)/2;       % link3
      0 0 L(4)/2;       % link4
      0 0 L(5)/2;       % link5
      0 0 L(6)/2 ];     % link6

% Inerzie (approssimazione)
Ixx = [0.05, 0.04, 0.02, 0.01, 0.005, 0.002];
Iyy = [0.06, 0.05, 0.025, 0.012, 0.006, 0.003];
Izz = [0.03, 0.02, 0.015, 0.008, 0.004, 0.002];

Ixy = zeros(1,6);
Ixz = zeros(1,6);
Iyz = zeros(1,6);

% Costruzione vettore p (6x10 → 60x1) preallocato
p = zeros(60,1);  % 6 link × 10 parametri = 60 elementi

for i = 1:6
    idx = (i-1)*10 + (1:10);  % indice per riempire 10 elementi per link
    p(idx) = [ m(i), m(i)*r(i,1), m(i)*r(i,2), m(i)*r(i,3), ...
               Ixx(i), Iyy(i), Izz(i), Ixy(i), Ixz(i), Iyz(i) ];
end



%% Initial conditions (can be overwritten by Simulink)
q0 = zeros(6,1);
qp0  = zeros(6,1);    
qpp0 = zeros(6,1);    

q   = q0;    % joint positions
qp   = qp0;    % joint velocities
qpp = qpp0;    %joint accelerations