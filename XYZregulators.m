clc, clear, close all;
g=9.81;
Iy=1;
A=[0 1 0 0;
   0 0 -g 0;
   0 0 0 1;
   0 0 0 0];
B=[0 0;
    0 0;
    0 0;
    0 1/Iy];
C = [1 0 0 0;
    0 0 1 0];
C= [1 0 0 0];
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


