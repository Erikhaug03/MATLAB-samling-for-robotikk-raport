%% ====== Robot arm med motion planing motion =======
clc; close all; clear;

%% =============== DH-parametere ==================
% Vi bruker 'offset' for å definere start posisjon til armen.
L(1) = Link('revolute', 'd', 1,   'a', 0,   'alpha',  pi/2, 'offset', 0, 'qlim', deg2rad([-180 180]));
L(2) = Link('revolute', 'd', 0.1, 'a', 0.6, 'alpha',     0, 'offset', deg2rad(120), 'qlim', deg2rad([-20 120]));
L(3) = Link('revolute', 'd', -0.1,'a', 0.6, 'alpha',     0, 'offset', deg2rad(-120), 'qlim', deg2rad([-130 -10]));
L(4) = Link('revolute', 'd', 0.1, 'a', 0.2, 'alpha',     0, 'offset', 0, 'qlim', deg2rad([-90 20]));

%% ============ Definering av roboten ==============
robot = SerialLink(L, 'name', 'RTB_4DOF');
q_home = [0 0 0 0]; % Start posisjon til armen, følger DH_parameterene.
qlim = deg2rad([-130 0]);

%% ============== Plot av robot armen ==============
figure;
robot.plot(q_home, 'workspace', [-2 2.5 -2 2.5 0 2.5]);
[X, Y] = meshgrid([-1 1], [0.5 1.5]);
Z = meshgrid([0.8 0.8]);
[X_y11, Y11] = meshgrid(-1, [0.5 0.6]);
[X_y12, Y12] = meshgrid(-0.9, [0.5 0.6]);
[Y_x11, X11] = meshgrid(0.5, [-1 -0.9]);
[X12, Y_x12] = meshgrid([-1 -0.9], 0.6);
[X_y21, Y21] = meshgrid(-0.9, [1.4 1.5]);
[X_y22, Y22] = meshgrid(-1, [1.4 1.5]);
[X21, Y_x21] = meshgrid([-1 -0.9], 1.4);
[X22, Y_x22] = meshgrid([-1 -0.9], 1.5);
[X_y31, Y31] = meshgrid(1, [0.5 0.6]);
[X_y32, Y32] = meshgrid(0.9, [0.5 0.6]);
[X31, Y_x31] = meshgrid([0.9 1], 0.6);
[X32, Y_x32] = meshgrid([0.9 1], 0.5);
[X_y41, Y41] = meshgrid(1, [1.4 1.5]);
[X_y42, Y42] = meshgrid(0.9, [1.4 1.5]);
[X41, Y_x41] = meshgrid([0.9 1], 1.4);
[X42, Y_x42] = meshgrid([0.9 1], 1.5);
Z1 = meshgrid([0 0.8]);
hold on;
surf(X, Y, Z);
surf(X_y11, Y11, Z1);
surf(X_y12, Y12, Z1);
surf(X11, Y_x11, Z1);
surf(X12, Y_x12, Z1);
surf(X_y21, Y21, Z1);
surf(X_y22, Y22, Z1);
surf(X21, Y_x21, Z1);
surf(X22, Y_x22, Z1);
surf(X_y31, Y31, Z1);
surf(X_y32, Y32, Z1);
surf(X31, Y_x31, Z1);
surf(X32, Y_x32, Z1);
surf(X_y41, Y41, Z1);
surf(X_y42, Y42, Z1);
surf(X41, Y_x41, Z1);
surf(X42, Y_x42, Z1);

%% ================ Vinkler for jtraj ==============
q_start = [0 0 0 0];
q_over_bord = [pi/2 -pi/3 pi/2 -pi/2];
q_bord_ende = [pi/2 -deg2rad(100) deg2rad(100) -pi/2];

%% ========= Definisjon av transformasjoner ========
T_start = robot.fkine(q_start);
T_over_bord = robot.fkine(q_over_bord);
T_bord_ende = SE3(0, 1.25, 0.8) * SE3.Rz(20, 'deg'); % robot.fkine(q_bord_ende); 
T_bord_nerme = SE3(0, 0.5, 0.8) * SE3.Rz(10, 'deg');

%% ========== Skriver ut transformasjoner ==========
disp('start pose'); disp(T_start);
disp('over bordet'); disp(T_over_bord);
disp('på enden av bordet'); disp(T_bord_ende);
disp('nermest på bordet'); disp(T_bord_nerme);

%% ========= Inverse Kinematikk(Cartesian) =========
N = 50;
mask = [1 1 1 0 0 1];
T_bord_ende_s = ctraj(T_over_bord, T_bord_ende, N);
q_bord_ende_T = robot.ikine(T_bord_ende_s, 'mask', mask, 'q', q_over_bord);
T_bord_nerme_s = ctraj(T_bord_ende, T_bord_nerme, N);
q_bord_nerme_T = robot.ikine(T_bord_nerme_s, 'mask', mask, 'q', q_bord_ende_T);
T_over_bord_s = ctraj(T_bord_nerme, T_over_bord, N);
q_over_bord_T = robot.ikine(T_over_bord_s, 'mask', mask, 'q', q_bord_nerme_T);

%% ============= Joint space bevegelse =============
[q1_traj, qd1_traj, qdd1_traj] = jtraj(q_start, q_over_bord, N);
% [q2_traj, qd2_traj, qdd2_traj] = jtraj(q_over_bord, q_bord_ende_s(1)', N);

%% =============== simulering av arm ===============
robot.plot(q1_traj, 'workspace', [-2 2.5 -2 2.5 0 2.5]);
robot.plot(q_bord_ende_T,'workspace', [-2 2.5 -2 2.5 0 2.5]);
robot.plot(q_bord_nerme_T, 'workspace', [-2 2.5 -2 2.5 0 2.5]);
robot.plot(q_over_bord_T, 'workspace', [-2 2.5 -2 2.5 0 2.5]);


