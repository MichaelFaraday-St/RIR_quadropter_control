function [syst, num, den, tf_MOO, K, Ke] = regulation_MOO(A,B,C,D,Q,R)
    % Define Q and R for LQR
    if width(A) == 4
        Q = diag([100, 30, 10, 1]);
        R = diag([1]);
    else
        Q = diag([100, 30]);
        R = diag([1]);
    end

    % LQR calculation for gain K
    [K, S, E] = lqr(A, B, Q, R);

    % Partition the A and B matrices for Observer Design
    Aaa = A(1,1);
    Aab = A(1,2:end);
    Aba = A(2:end,1);
    Abb = A(2:end,2:end);
    
    Ba = B(1,1);
    Bb = B(2:end,1);
    
    % Choose observer poles
    Je = 5*E(1:end-1);
    Ke = acker(Abb',Aab',Je')';
    
    Ka = K(1,1);
    Kb = K(1,2:end);
    
    % Build the observer-based system matrices
    Ax = Abb - Ke*Aab;
    Bx = Ax*Ke + Aba - Ke*Aaa;
    Fx = Bb - Ke*Ba;
    Cx = [zeros(1,3);eye(3,3)];
    Dx = [1; Ke];
    
    % Construct the augmented closed-loop (observer + feedback) system
    Av = Ax - Fx*Kb;
    Bv = Bx - Fx*(Ka + Kb*Ke);
    Cv = -Kb;
    Dv = -(Ka + Kb*Ke);
    
    % Obtain transfer function
    [num,den] = ss2tf(Av,Bv,-Cv,-Dv);
    tf_MOO = tf(num,den);

    % Create the state-space model
    syst = ss(Av, Bv, -Cv, -Dv);
end