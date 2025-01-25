clc, clear, close all;
% Define constants
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

% Define matrix A
A = [0 0 0 1 0 0 0 0  0 0 ;
     0 0 0 0 1 0 0 0  0 0 ;
     0 0 0 0 0 1 0 0  0 0 ;
     0 0 0 0 0 0 0 -g  0 0 ;
     0 0 0 0 0 0 -g 0  0 0 ;
     0 0 0 0 0 0 0  0 0 0 ;
     0 0 0 0 0 0 0  0 1 0 ;
     0 0 0 0 0 0 0  0 0 1 ;
     0 0 0 0 0 0 0  0 0 0 ;
     0 0 0 0 0 0 0  0 0 0];

% Define matrix B
B = [0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     0 0 0 0;
     -1/m -1/m -1/m -1/m;
     0 0 0 0;
     0 0 0 0;
     d/Ix 0 -d/Ix 0;
     0 d/Iy 0 -d/Iy];

% Define matrix C
C = [1 0 0 0 0 0 0 0 0 0  ;
     0 1 0 0 0 0 0 0 0 0  ;
     0 0 1 0 0 0 0 0 0 0  ;
     0 0 0 0 0 0 1 0 0 0  ;
     0 0 0 0 0 0 0 1 0 0  ];

% Define matrix D
D = zeros(5, 4);  % 6 rows for outputs and 4 columns for inputs
rank(ctrb(A, B))
rank(obsv(A,C))

% Select first input column (vertical thrust) for pole placement
B1 = B(:,1);  % Using only the first column of B (thrust input)

% Check controllability of the single-input system
ctrl_rank = rank(ctrb(A, B1));
fprintf('Controllability rank with single input: %d\n', ctrl_rank);

% Create state space model
myss = ss(A, B, C, D);
%current_poles = pole(myss);




% (2) Volba Q, R
Q = diag([ 10, 10, 10, ...
           2,  2,  2, ...
           20, 20, 10, ...
           1,  1,  1 ]);
R = diag([ 0.1, 0.1, 0.1, 0.1 ]);

Q=eye(10)
R= diag([1 1 1 1])
% (3) Výpočet LQR zisku
[K, S, E] = lqr(A, B, Q, R);

%IU = 1;  % Specify the input number you want to convert
%[num, den] = ss2tf(A, B, C, D, IU);





Aaa=A(1,1);
Aab=A(1,2:end);
Aba=A(2:end,1);
Abb=A(2:end,2:end);
Ba=B(1,1:end);
Bb=B(2:end,1:end);

Pe=[-1, -2, -5,-10,-12,-15,-7,-6,-20];
Ke1 = place(Abb',Aab',Pe')';



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

%prenos_MOO = tf(num,den)