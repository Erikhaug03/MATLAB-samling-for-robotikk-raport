clc; close all; clear;
import ETS3.*

% DH-parameterene
% Vi bruker 'offset' for å definere start posisjon til armen.
L(1) = Link('revolute', 'd', 1,   'a', 0,   'alpha',  pi/2, 'offset', 0, 'qlim', deg2rad([-180 180]));
L(2) = Link('revolute', 'd', 0.1, 'a', 0.6, 'alpha',     0, 'offset', deg2rad(120), 'qlim', deg2rad([-20 120]));
L(3) = Link('revolute', 'd', -0.1,'a', 0.6, 'alpha',     0, 'offset', deg2rad(-120), 'qlim', deg2rad([-130 0]));
L(4) = Link('revolute', 'd', 0.1, 'a', 0.2, 'alpha',     0, 'offset', 0, 'qlim', deg2rad([-90 20]));
robot = SerialLink(L, 'name', 'RTB_4DOF');

% Start posisjon for armen
q = [0 0 0 0];
robot.plot(q, 'workspace',[-2 2.5 -2 2.5 0 2.5]);
pause(3)

% Setter ønsket posisjon basert på startposisjon
T0 = robot.fkine(q); % Nåverende SE3-posisjon
p0 = transl(T0); % [x y z] verdiene fra T0              

% For eksempel rett over bordoverflaten vi skal vaske
p_d1 = p0 + [-0.5 1.2 -0.2]; % ønsket [x y z] verdier

% Loop parametere
k = 1;
dt = 0.01; % tid per steg
Tsim = 0.3; % simuleringstid
N = Tsim/dt; % steg = simuleringstid/(tid/steg)

for k = 2:N

    % Forwards kinematics
    T = robot.fkine(q);
    p = transl(T); % Nåverende posisjon

    % Posisjonsfeil
    e_pos = (p_d1 - p).';

    % Ønsket kartesisk posisjonshastighet
    kar_pos = k*e_pos;

    % Jacobi og leddhastighet
    J = robot.jacob0(q);
    Jpos = J(1:3, :);
    
    qdot = pinv(Jpos) * kar_pos;

    % rotasjon rundt z-akse
    total_rot = -pi/4;
    extra_qdot4 = total_rot / Tsim;  
    qdot(4) = qdot(4) + extra_qdot4;  

    % Setter sammen posisjonsendringen og rotasjonsendringen
    q = q + (qdot.' * dt);

    % Plotter roboten 
    robot.plot(q);
    
end

p_d2 = p_d1 + [0 0.2 -0.25];

for k = 1:N

   % Forwards kinematics
    T = robot.fkine(q);
    p = transl(T); % Nåverende posisjon

    % Posisjonsfeil
    e_pos = (p_d2 - p).';

    % Ønsket kartesisk posisjonshastighet
    kar_pos = k*e_pos;

    % Jacobi og leddhastighet
    J = robot.jacob0(q);
    Jpos = J(1:3, :);
    
    qdot = pinv(Jpos) * kar_pos;

    % rotasjon rundt z-akse
    total_rot = 0;
    extra_qdot4 = total_rot / Tsim;  
    qdot(4) = qdot(4) + extra_qdot4;  

    % Setter sammen posisjonsendringen og rotasjonsendringen
    q = q + (qdot.' * dt);

    % Plotter roboten 
    robot.plot(q);
    
end

p_d3 = p_d2 + [0 -0.6 0];

for k = 1:N

    % Forwards kinematics
    T = robot.fkine(q);
    p = transl(T); % Nåverende posisjon

    % Posisjonsfeil
    e_pos = (p_d3 - p).';

    % Ønsket kartesisk posisjonshastighet
    kar_pos = k*e_pos;

    % Jacobi og leddhastighet
    J = robot.jacob0(q);
    Jpos = J(1:3, :);
    
    qdot = pinv(Jpos) * kar_pos;

    % rotasjon rundt z-akse
    total_rot = -pi/4;
    extra_qdot4 = total_rot / Tsim;  
    qdot(4) = qdot(4) + extra_qdot4;  

    % Setter sammen posisjonsendringen og rotasjonsendringen
    q = q + (qdot.' * dt);

    % Plotter roboten 
    robot.plot(q);
end

p_d4 = p_d3 + [0 0.3 0.25];

for k = 1:N

    % Forwards kinematics
    T = robot.fkine(q);
    p = transl(T); % Nåverende posisjon

    % Posisjonsfeil
    e_pos = (p_d4 - p).';

    % Ønsket kartesisk posisjonshastighet
    kar_pos = k*e_pos;

    % Jacobi og leddhastighet
    J = robot.jacob0(q);
    Jpos = J(1:3, :);
    
    qdot = pinv(Jpos) * kar_pos;

    % rotasjon rundt z-akse
    total_rot = 0;
    extra_qdot4 = total_rot / Tsim;  
    qdot(4) = qdot(4) + extra_qdot4;  

    % Setter sammen posisjonsendringen og rotasjonsendringen
    q = q + (qdot.' * dt);

    % Plotter roboten 
    robot.plot(q);
end