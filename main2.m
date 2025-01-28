clc;
clear;
clf;
close all;
format compact

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

% Constants
% Radians to degree
RadianToDegree = 180/pi; 
% Degree to radians
DegreeToRadian = pi/180;

% Quadcopter parameters
Mass = 1.3;                 % [kg]
ArmLenght = 0.27;           % [m]
XMomentOfInertia =  0.023;  % [kg m^2]
YMomentOfInertia =  0.023;  % [kg m^2]
ZMomentOfInertia =  0.047;  % [kg m^2]

% Initial state of quadcopter
quadcopterInitState.BodyXYZPosition.X = 0;  % [m]
quadcopterInitState.BodyXYZPosition.Y = 0;  % [m]
quadcopterInitState.BodyXYZPosition.Z = -6; % [m]           
quadcopterInitState.BodyXYZVelocity.X = 0;            
quadcopterInitState.BodyXYZVelocity.Y = 0;            
quadcopterInitState.BodyXYZVelocity.Z = 0;
quadcopterInitState.BodyEulerAngle.Phi = 0;
quadcopterInitState.BodyEulerAngle.Theta = 0;
quadcopterInitState.BodyEulerAngle.Psi = 0;
quadcopterInitState.BodyAngularRate.dPhi = 0;
quadcopterInitState.BodyAngularRate.dTheta = 0;
quadcopterInitState.BodyAngularRate.dPsi = 0;

% Control variables - total thrust and moments on each control axis
quadcopterInitControlInputs = [0, 0, 0, 0]';     % (T, M1, M2, M3)
                           
% Initiate quadcopter
quadcopter = Quadcopter(Mass, ...               
               XMomentOfInertia, ...
               YMomentOfInertia, ...
               ZMomentOfInertia, ...
               quadcopterInitState, ...
               quadcopterInitControlInputs,...
               deltaT);
%% CHANGED
% Setup visualization
figure(1);
h = gca;
view(3);
grid on;
axis([-2 2 -2 2 -7 1]);
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
hold(h, 'on');
set(h, 'ZDir', 'reverse');

% Draw waypoints
plot3(wayPoints(:,1), wayPoints(:,2), wayPoints(:,3), 'r*');

% Drone visualization parameters
arm_length = ArmLenght;
rotor_radius = 0.1;
body_radius = 0.15;

currentWaypoint = 1;
SimulationEND = 0;

%% UNCHANGED
% Simulation
for i = 1 : deltaT : simulationTime
    t=i * deltaT
    % Action for total thrust
    quadcopter.TotalThrustControlAction(0);
    % Action for attitude
    quadcopter.AttitudeControlAction(0,0,0);
   
    % Update state of quadcopter
    quadcopter.UpdateState();

    % Get actual state of quadcopter
    quadcopterActualState = quadcopter.GetState(); 

    % Crash check
    if (quadcopterActualState.BodyXYZPosition.Z >= 0)
        msgbox('Quadcopter Crashed!', 'Error', 'error');
        break;
    end
%% CHANGED
    % Waypoint check
    [SimulationEND, currentWaypoint]=CheckWayPointTrack(...
                quadcopterActualState.BodyXYZPosition,...
                t,...
                timeForWaypointPasage,...
                wayPoints,...
                positionTolerance,...
                currentWaypoint);
    if (SimulationEND)
        break;
    end

    % Visualization
    cla(h);
    hold(h, 'on');
    
    % Current position and orientation
    pos = [quadcopterActualState.BodyXYZPosition.X, ...
           quadcopterActualState.BodyXYZPosition.Y, ...
           quadcopterActualState.BodyXYZPosition.Z];
    angles = [quadcopterActualState.BodyEulerAngle.Phi, ...
              quadcopterActualState.BodyEulerAngle.Theta, ...
              quadcopterActualState.BodyEulerAngle.Psi];
    
    % Draw drone
    drawDrone(h, pos, angles, arm_length, rotor_radius, body_radius);
    
    % Draw waypoints
    plot3(wayPoints(:,1), wayPoints(:,2), wayPoints(:,3), 'r*');
    
    % Update plot
    drawnow;
end
%% CHANGED
% Helper function to draw the drone
function drawDrone(h, pos, angles, arm_length, rotor_radius, body_radius)
    % Get rotation matrix
    R = RollPitchYaw2Rotation(angles);
    
    % Draw arms
    arms = arm_length * [1 0 0; 0 1 0; -1 0 0; 0 -1 0];
    for i = 1:4
        arm_end = pos + (R * arms(i,:)')';
        plot3(h, [pos(1) arm_end(1)], [pos(2) arm_end(2)], [pos(3) arm_end(3)], 'k-', 'LineWidth', 2);
    end
    
    % Draw rotors
    theta = 0:0.1:2*pi;
    for i = 1:4
        arm_end = pos + (R * arms(i,:)')';
        rotor_x = arm_end(1) + rotor_radius * cos(theta);
        rotor_y = arm_end(2) + rotor_radius * sin(theta);
        rotor_z = arm_end(3) * ones(size(theta));
        plot3(h, rotor_x, rotor_y, rotor_z, 'b-');
    end
    
    % Draw body
    [X,Y,Z] = sphere(20);
    X = body_radius * X;
    Y = body_radius * Y;
    Z = body_radius * Z;
    surf(h, X + pos(1), Y + pos(2), Z + pos(3), 'FaceColor', 'r', 'EdgeColor', 'none');
end
