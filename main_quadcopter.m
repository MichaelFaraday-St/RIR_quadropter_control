%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            % QUADCOPTER %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;
clf;
close all;
format compact

%% Load State Space Model and Regulation Parameters
run('maticeABC.m');

%% Waypoints Definition
% At these times, the quadcopter must pass the appropriate waypoint
timeForWaypointPasage = [100, 170, 300, 400, 500];  % [s]

% Waypoints matrix definition
wayPoints = [...
    0, 0, -6;        % [X, Y, Z] - waypoint in [m]
    1, 1, -6;
    2, 1, -7;
    0, 0, -5;
    1, 1, -6];

% Position tolerance
positionTolerance = 0.2;    % [m]

% Simulation parameters
deltaT = 0.01;              % [s]
simulationTime = max(timeForWaypointPasage) + 20; % [s]

%% Initialize States
x_x = zeros(4,1); % x,dx,theta,dtheta
x_y = zeros(4,1); % y,dy,phi,dphi 
x_z = zeros(2,1); % z,dz
x_z(1)=-6; % z0 = -6 [m]

x_hat_x = zeros(3,1); % measuring x, predicting dx,theta,dtheta
x_hat_y = zeros(3,1); % measuring y, predicting dy,phi,dphi
x_hat_z = zeros(1,1); % measuring z, predicting dz



%% Visualization Setup
figure(1);
h = gca;
view(3);
grid on;
axis([-2 3 -2 2 -7 1]); % axis view limits
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
hold(h, 'on');
set(h, 'ZDir', 'reverse'); % Z axis reversed

% Draw waypoints
plot3(wayPoints(:,1), wayPoints(:,2), wayPoints(:,3), 'r*', 'MarkerSize', 8, 'LineWidth', 2);
% drone body parametres for visualization
arm_length = 0.27;
rotor_radius = 0.1;
body_radius = 0.15;

currentWaypoint = 1; % index of finish point for the regulator
SimulationEND = 0;

% Initialize trajectory line
trajectory = animatedline('Color', 'b', 'LineWidth', 1.5);

%% Simulation Loop
for t = 0:deltaT:simulationTime
    % Get desired state from waypoint tracking
    if currentWaypoint <= size(wayPoints,1)
        %                                     x ,dx,theta,dtheta
        target_x = [wayPoints(currentWaypoint,1); 0; 0; 0];
        target_y = [wayPoints(currentWaypoint,2); 0; 0; 0];
        target_z = [wayPoints(currentWaypoint,3); 0];
    else
        target_x = zeros(4,1);
        target_y = zeros(4,1);
        target_z = zeros(2,1);
    end
    
    % Sensor measurements (only position)
    y_x = Cx * x_x;
    y_y = Cy * x_y;
    y_z = Cz * x_z;
    
    %% Compute Control Inputs (LQR)
    u_x = -K_x * ([x_x(1); x_hat_x] - target_x); % M2
    u_y = -K_y * ([x_y(1); x_hat_y] - target_y); % M1
    u_z = -K_z * ([x_z(1); x_hat_z] - target_z); % T (M3 = 0)
    
    %% Observer Update for MOO
    % dx_odhad = Ared * x_odhad + Bred * u(1) + Ke_x * (y(1) - Cred * x(1))
    x_hat_x_dot = Ax(2:end,2:end) * x_hat_x + Bx(2:end,:) * u_x + Ke_x * (y_x - Cx(:,1) * x_x(1));
    x_hat_x = x_hat_x + x_hat_x_dot * deltaT;

    x_hat_y_dot = Ay(2:end,2:end) * x_hat_y + By(2:end,:) * u_y + Ke_y * (y_y - Cy(:,1) * x_y(1));
    x_hat_y = x_hat_y + x_hat_y_dot * deltaT;

    x_hat_z_dot = Az(2:end,2:end) * x_hat_z + Bz(2:end,:) * u_z + Ke_z * (y_z - Cz(:,1) * x_z(1));
    x_hat_z = x_hat_z + x_hat_z_dot * deltaT;
    
    %% Update System State
    x_x_dot = Ax * x_x + Bx * u_x;
    x_y_dot = Ay * x_y + By * u_y;
    x_z_dot = Az * x_z + Bz * u_z;
    
    x_x = x_x + x_x_dot * deltaT;
    x_y = x_y + x_y_dot * deltaT;
    x_z = x_z + x_z_dot * deltaT;
    
    %% Check Waypoint
    if currentWaypoint <= size(wayPoints,1)
        positionError = norm([x_x(1), x_y(1), x_z(1)] - wayPoints(currentWaypoint,:));
        if positionError < positionTolerance
            fprintf("Waypoint %d reached at time %.2f seconds.\n", currentWaypoint, t);
            % disp(wayPoints(currentWaypoint,:))
            fprintf("Deviation: X: %.3f m, Y: %.3f m, Z: %.3f m\n", x_x(1) - wayPoints(currentWaypoint,1), x_y(1) - wayPoints(currentWaypoint,2), x_z(1) - wayPoints(currentWaypoint,3));
            currentWaypoint = currentWaypoint + 1;
        end
    elseif currentWaypoint > size(wayPoints,1)
        break;
    end
    
    %% Visualization Update
    addpoints(trajectory, x_x(1), x_y(1), x_z(1));
    pos = [x_x(1), x_y(1), x_z(1)];
    angles = [-x_y(3), -x_x(3), 0]; % Upravit dle vašich stavů, např. [phi, theta, psi]
    droneParts = drawDrone(h, pos, angles, arm_length, rotor_radius, body_radius);
    drawnow;
    if t < simulationTime
        delete(droneParts);
    end
end

%% Helper function
function droneParts = drawDrone(h, pos, angles, arm_length, rotor_radius, body_radius)
    droneParts = [];
    % Get rotation matrix
    R = RollPitchYaw2Rotation(angles);
    
    % Draw arms
    arms = arm_length * [1 0 0; 0 1 0; -1 0 0; 0 -1 0];
    for i = 1:4
        arm_end = pos + (R * arms(i,:)')';
        armPlot = plot3(h, [pos(1) arm_end(1)], [pos(2) arm_end(2)], [pos(3) arm_end(3)], 'k-', 'LineWidth', 2);
        droneParts = [droneParts; armPlot];
    end
    
    % Draw rotors
    theta = 0:0.1:2*pi;
    for i = 1:4
        arm_end = pos + (R * arms(i,:)')';
        rotor_x = arm_end(1) + rotor_radius * cos(theta);
        rotor_y = arm_end(2) + rotor_radius * sin(theta);
        rotor_z = arm_end(3) * ones(size(theta));
        rotorPlot = plot3(h, rotor_x, rotor_y, rotor_z, 'b-');
        droneParts = [droneParts; rotorPlot];
    end
    
    % Draw body
    [X,Y,Z] = sphere(20);
    X = body_radius * X;
    Y = body_radius * Y;
    Z = body_radius * Z;
    bodySurf = surf(h, X + pos(1), Y + pos(2), Z + pos(3), 'FaceColor', 'r', 'EdgeColor', 'none');
    droneParts = [droneParts; bodySurf];
end
