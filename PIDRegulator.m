classdef PIDRegulator < handle
    % Example of PID regulator
    properties
       kP 
       kI 
       kD
    end

    properties
       error = 0.0;             % Current error
       errorSum = 0.0;          % Sum of error
       prevError = 0.0;         % Previous error
       currentAction = 0.0;     % Current action
    end

    methods
        % Regulator initiate
        function obj = PIDRegulator(Kp, Ki, Kd)
            obj.kP = Kp;
            obj.kI = Ki;
            obj.kD = Kd;
        end

        % Calculate action
        function obj = CalculateAction(obj, actualValue, desiredValue, dt)
            obj.error = desiredValue - actualValue;
            obj.currentAction = obj.kP * obj.error + ...
                     obj.kI * obj.errorSum + ...
                     (obj.kD * (obj.error - obj.prevError) / dt);
            obj.prevError = obj.error;
            obj.errorSum = obj.errorSum + obj.error;
        end
    
        % Get action
        function action = GetCurrentAction(obj)
            action = obj.currentAction;
        end
    end
end