clc, clear, close all;

%% Define constants

g = 9.81;  % Gravitational acceleration (you can change it if needed)
Mass = 1.3;
d= 0.27;
XMomentOfInertia =  0.023;  % [kg m^2]
YMomentOfInertia =  0.023;  % [kg m^2]
ZMomentOfInertia =  0.047;  % [kg m^2]

m = Mass;
Ix = XMomentOfInertia;    % Moment of inertia about x-axis (set the actual value)
Iy = YMomentOfInertia;    % Moment of inertia about y-axis (set the actual value)
Iz = ZMomentOfInertia;    % Moment of inertia about z-axis (set the actual value)

%% Define matrixes

% Matrixes for X axes
Ax = [0 1 0 0;  % matrix A for X axis
      0 0 -g 0;
      0 0 0 1;
      0 0 0 0];

Bx = [0;  % matrix B for X axis
      0;
      0;
      1/Iy];

Cx = [1 0 0 0]; % matrix C for X axis - only X on the output

Dx = zeros(4,1); % matrix D for X axis

% Matrixes for Y axes
Ay = [0 1 0 0;  % matrix A for Y axis
      0 0 -g 0;
      0 0 0 1;
      0 0 0 0];

By = [0;  % matrix B for Y axis
      0;
      0;
      1/Ix];

Cy = [1 0 0 0]; % matrix C for Y axis - only Y on the output

Dy = zeros(4,1); % matrix D for Y axis

% Matrixes for Z axes
Az = [0 1;  % matrix A for Z axis
      0 0];

Bz = [0; 1/m]; % matrix B for Z axis

Cz = [1 0]; % matrix C for Z axis - only Z on the output

Dz = [0]; % matrix D for Z axis

%% Test of controllability and observability
checkControllabilityObservability(Ax, Bx, Cx)
checkControllabilityObservability(Ay, By, Cy)
checkControllabilityObservability(Az, Bz, Cz)

%% Calculating the parametres for regulation
[syst_x, num_x, den_x, tf_MOO_x, K_x, Ke_x] = regulation_MOO(Ax,Bx,Cx,Dx);
[syst_y, num_y, den_y, tf_MOO_y, K_y, Ke_y] = regulation_MOO(Ay,By,Cy,Dy);
[syst_z, num_z, den_z, tf_MOO_z, K_z, Ke_z] = regulation_MOO(Az,Bz,Cz,Dz);
