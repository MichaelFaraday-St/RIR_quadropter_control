clc, clear, close all;
deltaT = 0.01;
timeForWaypointPasage = [100, 170, 300, 400, 500];
% totalTime = sum(timeForWaypointPasage);
totalTime = 20;

% Waypoints matrix definition
wayPoints = [...
    0, 0, -6;        % [X, Y, Z] - waypoint in [m]
    1, 1, -6;
    2, 1, -7;
    0, 0, -5;
    1, 1, -6];

% Position tolerance
positionTolerance = 0.2;    % [m]

% Quadcopter parameters
Mass = 1.3;                 % [kg]
ArmLenght = 0.27;           % [m]
XMomentOfInertia =  0.023;  % [kg m^2]
YMomentOfInertia =  0.023;  % [kg m^2]
ZMomentOfInertia =  0.047;  % [kg m^2]
g=9.81;
Iy=0.023;
A=[0 1 0 0;
   0 0 -g 0;
   0 0 0 1;
   0 0 0 0];
B=[0 0;
    0 0;
    0 0;
    0 1/Iy];

% C = [1 0 0 0;
%     0 0 1 0];

C= [1 0 0 0]; % only X on the output
D=zeros(4,2);

Q = eye(4);
R=eye(2);

[K,s,J] = lqr(A,B,Q,R);

Aaa=A(1,1);
Aab=A(1,2:end);
Aba=A(2:end,1);
Abb=A(2:end,2:end);
Ba=B(1,1:end);
Bb=B(2:end,1:end);

Pe=5*J(1:end-1);
%Pe=[-1, -2, -5,-10,-12,-15,-7,-6,-20];
Ke = acker(Abb',Aab',Pe)';

Astr=Abb-Ke*Aab;
Bstr=Astr*Ke+Aba-Ke*Aaa;
Fstr=Bb-Ke*Ba;
Cstr=[0 0 0 0 0 0 0 0 0;eye(9)];
Dstr=[1;Ke];

Ka=K(1:end,1);
Kb=K(1:end,2:end);


Avln = Astr - Fstr*Kb;
Bvln = Bstr-Fstr*(Ka+Kb*Ke);
Cvln = -Kb;
Dvln = -(Ka+Kb*Ke);

[num,den] = ss2tf(Avln,Bvln,-Cvln,-Dvln);


% Run the simulation:
out = sim('untitled5.slx');

% Access time and data from Xposition (timeseries)
time_x   = out.Xposition.Time;
X_data   = out.Xposition.Data;

% Access time and data from T_M2 (timeseries)
TM2_data = out.T_M2.Data;        % could be Nx2 if you have 2 columns, etc.
T_data  = TM2_data(:,1);
M2_data = TM2_data(:,2);


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
quadcopterInitControlInputs = [Mass*g, 0, 0, 0]';     % (T, M1, M2, M3)
                           
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
% axis([-2 2 -2 2 -7 1]);
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
for i = 1 : (20/deltaT)
    t=i * deltaT;
    disp(t)
    disp([quadcopter.TotalThrust quadcopter.Moments(2) quadcopter.parameters.BodyXYZPosition.X])
    quadcopterActualState = quadcopter.GetState(); 
    % Action for total thrust
    quadcopter.TotalThrustControlAction(T_data(i)+Mass*g/(cos(quadcopterActualState.BodyEulerAngle.Phi)*cos(quadcopterActualState.BodyEulerAngle.Theta)));
    
    % Action for attitude
    quadcopter.AttitudeControlAction(0,M2_data(i),0);
   
    % Update state of quadcopter
    quadcopter.UpdateState();

    % Get actual state of quadcopter
    quadcopterActualState = quadcopter.GetState(); 

    % % Crash check
    % if (quadcopterActualState.BodyXYZPosition.Z >= 0)
    %     msgbox('Quadcopter Crashed!', 'Error', 'error');
    %     break;
    % end
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