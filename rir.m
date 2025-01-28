clc;
clear;
close all;

g = 9.81;
phi = 0;
theta = 0; 

A = [0  0  0  1  0  0  0  0  0  0                 0                   0;
     0  0  0  0  1  0  0  0  0  0                 0                   0;
     0  0  0  0  0  1  0  0  0  0                 0                   0;
     0  0  0  0  0  0  0  g*cos(theta) 0  0       0                   0;
     0  0  0  0  0  0 -g*cos(phi) 0   0  0       0                   0;
     0  0  0  0  0  0  0  0        0  0          0                   0;
     0  0  0  0  0  0  0  0        0  1          sin(phi)*tan(theta) cos(phi)*tan(theta);
     0  0  0  0  0  0  0  0        0  0          cos(phi)            -sin(phi);
     0  0  0  0  0  0  0  0        0  0          sin(phi)/cos(theta) cos(phi)/cos(theta);
     0  0  0  0  0  0  0  0        0  0          0                   0;
     0  0  0  0  0  0  0  0        0  0          0                   0;
     0  0  0  0  0  0  0  0        0  0          0                   0];

m = 1.3;       % Hmotnosť kvadrokoptéry [kg]
Ix = 0.023;    % Moment zotrvačnosti okolo osi X [kg*m^2]
Iy = 0.023;    % Moment zotrvačnosti okolo osi Y [kg*m^2]
Iz = 0.047;    % Moment zotrvačnosti okolo osi Z [kg*m^2]

B = [0     0     0     0;
     0     0     0     0;
     0     0     0     0;
     sin(theta)/m   0     0     0;
    -sin(phi)/m     0     0     0;
     cos(phi)*cos(theta)/m  0     0     0;
     0     0     0     0;
     0     0     0     0;
     0     0     0     0;
     0   1/Ix    0     0;
     0     0   1/Iy    0;
     0     0     0   1/Iz];

% Matica C (výstupná matica)
C = [1  0  0  0  0  0  0  0  0  0  0  0; % X - poloha
     0  1  0  0  0  0  0  0  0  0  0  0; % Y - poloha
     0  0  1  0  0  0  0  0  0  0  0  0; % Z - poloha
     0  0  0  0  0  0  1  0  0  0  0  0; % Phi (uhol naklonenia okolo X)
     0  0  0  0  0  0  0  1  0  0  0  0; % Theta (uhol naklonenia okolo Y)
     0  0  0  0  0  0  0  0  1  0  0  0];% Psi (uhol natočenia okolo Z)

% Matica D (priamy prenos)
D = zeros(6, 4);

% Váhy pre LQR regulátor
Q = diag([1000, 1000, 2000, 100, 100, 100, 100, 100, 100, 1, 1, 1]);
R = diag([10, 10, 10, 10]);% Váha pre vstupy

% Výpočet LQR zisku
K = lqr(A, B, Q, R);

J = [-30, -35, -40, -45, -50, -55, -60, -65, -70, -75, -80, -85];

Ke = place(A', C', J)';

% Test stability
%eig_A_BK = eig(A - B * K);
%disp('Vlastné hodnoty systému A - B * K:');
%disp(eig_A_BK);

x = [0; 0; -6; 0; 0; 0; 0; 0; 0; 0; 0; 0];
x_hat = zeros(12, 1);

% At these times, the quadcopter must pass the appropriate waypoint
timeForWaypointPasage = [100,200,300,400,500]; % [s]

% Waypoints - these points must be flown by quadcopter
wayPoints = [0 0 -6;        % [X, Y, Z] - waypoint in [m]
             1 1 -6;
             1 0 -5;
             2 1 -5;
             1 1 -6];
% Position tolerance
positionTolerance = 0.1;    % [m]

% Simulation parameters
deltaT = 0.01;              % [s]
simulationTime = max(timeForWaypointPasage) + 20; % [s]

time_log = [];
trajectory_log = []; % Pozície pre 3D graf

current_waypoint = 1;

x(3) = wayPoints(1, 3); % Počiatočný stav pre z
x_hat(3) = wayPoints(1, 3);

% Inicializácia 3D grafu
figure;
scatter3(wayPoints(:, 1), wayPoints(:, 2), wayPoints(:, 3), 'ro', 'filled', 'DisplayName', 'Waypointy');
hold on;
traj_plot = plot3(0, 0, 0, 'b', 'DisplayName', 'Trajektória');
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('3D trajektória kvadrokoptéry');
legend;
grid on;
xlim([-1 2]);
ylim([-1 2]);
zlim([-7 1]);

k = 0;
% Simulation
for t = 0:deltaT:simulationTime
    
    phi = x(7);
    theta = x(8);
    
    A(4,8) = g * cos(theta);
    A(5,7) = -g * cos(phi); 
    A(7,11) = sin(phi) * tan(theta);
    A(7,12) = cos(phi) * tan(theta);
    A(8,11) = cos(phi);
    A(8,12) = -sin(phi);
    A(9,11) = sin(phi)/cos(theta);
    A(9,12) = cos(phi)/cos(theta);

    B(4,1) = sin(theta) / m;
    B(5,1) = -sin(phi) / m; 
    B(6,1) = cos(phi) * cos(theta) / m;


    % Výpočet aktuálneho výstupu
    y = C * x;

    % Pozorovateľ - aktualizácia odhadovaného stavu
    x_hat_dot = A * x_hat + B * (-K * (x_hat - [wayPoints(current_waypoint, :)'; zeros(9, 1)])) + Ke * (y - C * x_hat);
    x_hat = x_hat + x_hat_dot * deltaT;
    
    % Regulátor - cieľový stav
    if current_waypoint <= size(wayPoints, 1)
        target = [wayPoints(current_waypoint, :)'; zeros(9, 1)]; % Pozícia + nulové rýchlosti/orientácie
        error = x_hat - target; % Chyba
        
        % Výpočet riadiaceho vstupu
        u = -K * error; % Stavová spätná väzba

    else
        u = zeros(4, 1); % Ak sú všetky waypointy splnené
    end

    % Aktualizácia reálneho stavu systému
    x_dot = A * x + B * u;
    x = x + x_dot * deltaT;

    % Crash check
    if x(3) >= 0
        disp('Kvadrokoptéra narazila');
        break;
    end

    % Kontrola waypointov
    if current_waypoint <= size(wayPoints, 1)
    distance_to_waypoint = norm(x(1:3) - wayPoints(current_waypoint, :)');
    
    if distance_to_waypoint < positionTolerance
        disp(['Waypoint ', num2str(current_waypoint), ' dosiahnutý.']);
        fprintf("Waypoint %d dosiahnutý v čase %3.2f sekúnd.\n", current_waypoint, t);
        current_waypoint = current_waypoint + 1;
        
        % Ak sú všetky waypointy splnené, ukončime simuláciu
        if current_waypoint > size(wayPoints, 1)
            disp('Všetky waypointy dosiahnuté.');
            break;
        end
    end
end


    % Logovanie dát
    time_log = [time_log; t];
    trajectory_log = [trajectory_log; x(1), x(2), x(3)];

    %3D trajektória
    k = k+1;
    % Aktualizácia 3D grafu
    set(traj_plot, 'XData', trajectory_log(:, 1), ...
                   'YData', trajectory_log(:, 2), ...
                   'ZData', trajectory_log(:, 3));
    drawnow;
    
 end 
