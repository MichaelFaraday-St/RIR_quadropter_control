function checkControllabilityObservability(A, B, C)
    % Compute the controllability matrix and rank
    Co = ctrb(A, B);
    controllability_rank = rank(Co);
    
    % Compute the observability matrix and rank
    Ob = obsv(A, C);
    observability_rank = rank(Ob);
    
    % Number of states in the system
    num_states = size(A, 1);
    
    % Check controllability
    if controllability_rank == num_states
        fprintf('The system is CONTROLLABLE.\n');
    else
        fprintf('The system is NOT CONTROLLABLE.\n');
    end

    % Check observability
    if observability_rank == num_states
        fprintf('The system is OBSERVABLE.\n');
    else
        fprintf('The system is NOT OBSERVABLE.\n');
    end
end