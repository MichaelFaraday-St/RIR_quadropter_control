clc, clear, close all;
g=9.81;
Mass = 1.3;                 % [kg]
ArmLenght = 0.27;           % [m]
XMomentOfInertia =  0.023;  % [kg m^2]
YMomentOfInertia =  0.023;  % [kg m^2]
ZMomentOfInertia =  0.047;  % [kg m^2]
m=Mass;
Iy=YMomentOfInertia;
Ix=XMomentOfInertia;

%non linear models ABCD
Anonlin = [0 1 0 0;
           0 0 g*cos(theta) 0
           0 0 0 1;
           0 0 0 0];
Bnonlin = [0 0;
           0 0;
           0 0;
           0 1/Iy];
%
A=[0 1 0 0;
   0 0 g 0;
   0 0 0 1;
   0 0 0 0];
B=[0 0;
    0 0;
    0 0;
    0 1/Iy];
% C = [1 0 0 0;
%     0 0 1 0];
C= [1 0 0 0]; % vystup pouze x=x(1)
D=zeros(1,2);

Q = diag([1.8 1 1.5 1]); %eye(4);
R=diag([1 1]);
% Q = eye(size(A,2));
% R = eye(size(B,2)); % automaticky vytvori jednick. matice s dobrymi rozmery

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

[num,den] = ss2tf(Avln,Bvln,-Cvln,-Dvln); %manualne vypocitane

[k1,ke1,sys1,num1,den1] = myLQGdesign(A,B,C,D); % vypocitane pomoci funkce (stejne hodnoty)

Ay = [0 1 0 0;
      0 0 g 0;
      0 0 0 1;
      0 0 0 0];
By=[0 0;
    0 0;
    0 0;
    0 1/Ix];
Cy= [1 0 0 0];
Dy=zeros(1,2);

[ky,key,sysy,numy,deny] = myLQGdesign(Ay,By,Cy,Dy);

Az = [0 1;
      0 0];
Bz = [0;1/m];
Cz=[1 0];
Dz = 0;
[kz,kez,sysz,numz,denz] = myLQGdesign(Ay,By,Cy,Dy);


function [K, Ke, sys, num, den] = myLQGdesign(A, B, C, D)
    % MYLQGDESIGN  Computes the LQR gain (K), the observer gain (Ke),
    %              the transfer function (G) in tf form, and
    %              the state-space system (sys) for the final design.
    %
    %   [K, Ke, G, sys] = MYLQGDESIGN(A,B,C,D)
    %
    %   Inputs:
    %       A, B, C, D : State-space matrices describing the plant
    %                    (for a 4x4 A and B with 2 inputs)
    %
    %   Outputs:
    %       K          : LQR gain matrix
    %       Ke         : Observer (estimator) gain matrix
    %       G          : Transfer function of the final system
    %       sys        : State-space model (ss) of the final system
    %
    %   This function is adapted from your script to design both
    %   the full-state feedback (LQR) and the observer gains,
    %   then form the resulting closed-loop system.

    % Example weighting matrices for LQR (tune/modify as needed):
    Q = eye(size(A,2));
    R = eye(size(B,2));

    % -- 1) Compute the LQR gain K --
    [K, S, e] = lqr(A, B, Q, R); 
    %   K is the LQR gain
    %   S is the solution to the Riccati equation
    %   e are the closed-loop eigenvalues (A - B*K)

    % -- 2) Partition the A and B matrices for Observer Design (based on your script) --
    Aaa = A(1,1);
    Aab = A(1,2:end);
    Aba = A(2:end,1);
    Abb = A(2:end,2:end);

    Ba  = B(1, :);
    Bb  = B(2:end, :);

    % -- 3) Choose observer poles. Example: place them at 5x the magnitudes 
    %        of some closed-loop poles from e(1:end-1). Adjust as needed. --
    %        (This is just an example from your script logic.)
    Pe = 5 * e(1:end-1);

    % -- 4) Compute the observer gain K_e via pole placement (acker) --
    Ke = acker(Abb', Aab', Pe).';

    % -- 5) Build the observer-based system matrices as in your script --
    Astr = Abb - Ke*Aab;
    Bstr = Astr*Ke + Aba - Ke*Aaa;
    Fstr = Bb - Ke*Ba;

    % Extract K's parts for convenience:
    Ka = K(:,1);        % Gains w.r.t. state #1
    Kb = K(:,2:end);    % Gains w.r.t. states #2..end

    % -- 6) Construct the augmented closed-loop (observer + feedback) system --
    Avln = Astr - Fstr*Kb;               % New A
    Bvln = Bstr - Fstr*(Ka + Kb*Ke);     % New B
    Cvln = -Kb;                          % New C
    Dvln = -(Ka + Kb*Ke);                % New D

    % -- 7) Obtain transfer function from this representation --
    %   Note the sign convention in your script used -Cvln and -Dvln:
    [num, den] = ss2tf(Avln, Bvln, -Cvln, -Dvln);

    % Create the transfer function model
    %G = tf(num, den);

    % Create the state-space model
    sys = ss(Avln, Bvln, -Cvln, -Dvln);

end



