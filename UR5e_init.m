% ==========================================================
% Industrial Robotics
% Dynamic model of UR5e
% Adaptation of symbolic 2R exercise to 6-DOF robot
%
% Control developed in Simulink
% ==========================================================

clear all;
clc;

%% Robotics Toolbox initialization
startup_rvc

% Definizione del robot
L = [0.1625, 0.425, 0.3922, 0.1333, 0.0997, 0.0996];
L1 = Link('d',L(1),'a',0,'alpha',pi/2,'m',3.4,'r',[0 0 L(1)/2],'I',[0.01 0.01 0.01 0 0 0]);
L2 = Link('d',0,'a',L(2),'alpha',0,'m',8.6,'r',[L(2)/2 0 0],'I',[0.1 0.1 0.1 0 0 0]);
L3 = Link('d',0,'a',L(3),'alpha',0,'m',2.7,'r',[L(3)/2 0 0],'I',[0.05 0.05 0.05 0 0 0]);
L4 = Link('d',L(4),'a',0,'alpha',pi/2,'m',1.7,'r',[0 0 L(4)/2],'I',[0.02 0.02 0.02 0 0 0]);
L5 = Link('d',L(5),'a',0,'alpha',-pi/2,'m',1.0,'r',[0 0 L(5)/2],'I',[0.01 0.01 0.01 0 0 0]);
L6 = Link('d',L(6),'a',0,'alpha',0,'m',0.5,'r',[0 0 L(6)/2],'I',[0.005 0.005 0.005 0 0 0]);

Rob1 = SerialLink([L1 L2 L3 L4 L5 L6],'name','UR5e');
Rob1.base = [1 0 0 0; 0 -1 0 0; 0 0 -1 0.1625; 0 0 0 1];

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

%% ==========================
% DYNAMIC MODEL
% ==========================

% Inertia matrix B(q)
B = Rob1.inertia(q(:)');      % 6x6

% Coriolis / centrifugal matrix C(q, qp)
C = Rob1.coriolis(q(:)', qp(:)'); % 6x6

% Gravity vector g(q)
g = Rob1.gravload(q(:)');     % 6x1

%% ==========================
% Equation of motion
% tau = B(q) qdd + C(q,qd) qd + g(q)
% ==========================

tau = B*qpp + C*qp + g;

%% ==========================
% Save variables for Simulink
% ==========================

save('UR5e_dyn_data', ...
     'Rob1', ...
     'B', 'C', 'g', 'tau', ...
     'q', 'qp', 'qpp');

