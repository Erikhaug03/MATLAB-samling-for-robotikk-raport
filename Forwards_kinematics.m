%% Forwards Kinematics

clc; clear; close all;

syms theta1 theta2 theta3 theta4; % Leddvinkler


% Definere transformer for leddene
TB1 = [cos(theta1)   0  sin(theta1)               0 
       sin(theta1)   0 -cos(theta1)               0
       0             1            0           0.125
       0             0            0              1];

T12 = [cos(theta2)   -sin(theta2) 0  0.6*cos(theta2) 
       sin(theta2)   cos(theta2)  0 0.6*sin(theta2)
       0             0            1             0.1
       0             0            1              1];

T23 = [cos(theta3)   -sin(theta3) 0  0.6*cos(theta3) 
       sin(theta3)   cos(theta3)  0 0.6*sin(theta3)
       0             0            1            -0.1
       0             0            1              1];

T34 = [cos(theta4)   -sin(theta4) 0  0.2*cos(theta4)
       sin(theta4)   cos(theta4)  0 0.2*sin(theta4)
       0             0            1             0.1
       0             0            1              1];


TB4 = TB1*T12*T23*T34;

disp(simplify(TB4))

%% Peter corke toolbox for matlab

syms theta1 theta2 theta3 theta4
% DH-parameterene
L(1) = Link('revolute', 'd', 1,   'a', 0,   'alpha',  (pi)/2, ...
    'offset', 0, 'qlim', deg2rad([-180 180]));
L(2) = Link('revolute', 'd', 0.1, 'a', 0.6, 'alpha',     0, ...
    'offset', deg2rad(120), 'qlim', deg2rad([-20 120]));
L(3) = Link('revolute', 'd', -0.1,'a', 0.6, 'alpha',     0, ...
    'offset', deg2rad(-120), 'qlim', deg2rad([-130 -10]));
L(4) = Link('revolute', 'd', 0.1, 'a', 0.2, 'alpha',     0, ...
    'offset', 0, 'qlim', deg2rad([-90 20]));
robot = SerialLink(L, 'name', 'RTB_4DOF');

q = [theta1 theta2 theta3 theta4];

% Transformasjon for vert ledd
T01 = robot.A(1, q).T;
T12 = robot.A(2, q).T;
T23 = robot.A(3, q).T;
T34 = robot.A(4, q).T;

% Total transformasjons matrise
T04 = robot.fkine(q);
T04 = simplify(T04.T);

%% Numerisk forward kinematics

q_num = [0 0 0 0];
T04_num = robot.fkine(q_num);
T04_num = simplify(T04_num)

%% Differential kinematics

% Vi ønsker å ha en start posisjon med liddvinkelene fra DH-parameterene:
q_num = [0 0 0 0]; 

% Fra disse ledvinklene får vi jacobi matrisene:
J0 = robot.jacob0(q_num) % jacobi matrisen til base-rammen

%% Inverse kinematic

t_s = SE3(0.1, 1, 1);
mask = [1 1 1 0 0 1];
q_ikine = robot.ikine(t_s, 'mask', mask, 'q', q_num);
q_deg = rad2deg(q_ikine);



%% Simulering

T1 = SE3(0.5, -0.1, 1.5) * SE3.Rz(0, 'deg');
T2 = SE3(0, 1, 1) * SE3.Rz(0, 'deg');
N = 50;
mask = [1 1 1 0 0 1];
T_S = ctraj(T1, T2, N);
q1 = robot.ikine(T_S, 'mask', mask, 'q', q_num);
figure;
robot.plot(q1, 'workspace', [-2 2.5 -2 2.5 0 2.5]);

