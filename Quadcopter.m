classdef Quadcopter < handle
    % Quadcopter model for RIR students
    % J. Kovar
    properties
        physicalParameters
        parameters

        g                   % Gravity
        t                   % Actual simulaton time
        dt                  % Time step
        tf                  % Final simulation time
        
        state               % State vector
        changeState         % Change state vector
        
        TotalThrust         % total thrust
        Moments             % [moment1, moment2, moment3]
    end
    
    methods        
        function obj = Quadcopter( ...
                             mass, ...                             
                             xMomentOfInertia, ...
                             yMomentOfInertia, ...
                             zMomentOfInertia, ...
                             quadcopterInitState, ...
                             initInputs,...
                             deltaTime)
            
            % Mass
            obj.physicalParameters.Mass = mass;

            % Inertia matrix
            obj.physicalParameters.I = ...
                    [xMomentOfInertia,  0,                    0; ...
                     0,                 yMomentOfInertia,     0; ...
                     0,                 0,                    zMomentOfInertia];
            
            % Physical constants
            obj.g = 9.81;           % Gravity
            
            % Simulation parameters
            obj.t = 0.0;            % Simulation time
            obj.dt = deltaTime;     % Simulation delta time

            % Initial state 
            obj.parameters = quadcopterInitState;
            
            % States
            obj.state = zeros(12,1);
            obj.changeState = zeros(12,1);
            
            % Initial control vector
            obj.TotalThrust = initInputs(1);
            obj.Moments = initInputs(2:4);
        end
        
        % Returns current state of quadcopter
        function state = GetState(obj)
            state = obj.parameters;
        end
        
        % Calculate change of state vector
        function obj = CalculateNewState(obj)
            % Roll, pitch, yaw to rotation matrix
            rotation = RollPitchYaw2Rotation([obj.parameters.BodyEulerAngle.Phi obj.parameters.BodyEulerAngle.Theta obj.parameters.BodyEulerAngle.Psi]');            
            % Converts vector in body-fixed frame to inertial reference frame
            R = rotation';                                 
            
            % Build dx array from current state
            obj.changeState(1:3) = [obj.parameters.BodyXYZVelocity.X obj.parameters.BodyXYZVelocity.Y obj.parameters.BodyXYZVelocity.Z]'; 
            obj.changeState(4:6) = 1 / obj.physicalParameters.Mass * ([0; 0; obj.physicalParameters.Mass * obj.g] + R * obj.TotalThrust * [0; 0; -1]);
            obj.changeState(7:9) = [1  sin(obj.parameters.BodyEulerAngle.Phi)*tan(obj.parameters.BodyEulerAngle.Theta)  cos(obj.parameters.BodyEulerAngle.Phi)*tan(obj.parameters.BodyEulerAngle.Theta); ...
                                    0  cos(obj.parameters.BodyEulerAngle.Phi)                   -sin(obj.parameters.BodyEulerAngle.Phi); ...
                                    0  sin(obj.parameters.BodyEulerAngle.Phi)*sec(obj.parameters.BodyEulerAngle.Theta)  cos(obj.parameters.BodyEulerAngle.Phi)*sec(obj.parameters.BodyEulerAngle.Theta)] * [obj.parameters.BodyAngularRate.dPhi  obj.parameters.BodyAngularRate.dTheta obj.parameters.BodyAngularRate.dPsi]';
            obj.changeState(10:12) = ...
                (obj.physicalParameters.I) \ ...
                (obj.Moments - ...
                cross([obj.parameters.BodyAngularRate.dPhi obj.parameters.BodyAngularRate.dTheta obj.parameters.BodyAngularRate.dPsi]',... 
                obj.physicalParameters.I *...
                [obj.parameters.BodyAngularRate.dPhi obj.parameters.BodyAngularRate.dTheta obj.parameters.BodyAngularRate.dPsi]'));
        end
        
        %Updating state based on equations of motion
        function obj = UpdateState(obj)
            % Increase time 
            obj.t = obj.t + obj.dt;
            
            % Calculate new state (change)
            obj.CalculateNewState();

            % Calculate common state
            obj.state = [cell2mat(struct2cell(obj.parameters.BodyXYZPosition)); ...
                     cell2mat(struct2cell(obj.parameters.BodyXYZVelocity)); ...
                     cell2mat(struct2cell(obj.parameters.BodyEulerAngle)); ...
                     cell2mat(struct2cell(obj.parameters.BodyAngularRate))];
            obj.state = obj.state + obj.changeState.*obj.dt;

            obj.parameters.BodyXYZPosition.X = obj.state(1);
            obj.parameters.BodyXYZPosition.Y = obj.state(2);
            obj.parameters.BodyXYZPosition.Z = obj.state(3);
            obj.parameters.BodyXYZVelocity.X = obj.state(4);
            obj.parameters.BodyXYZVelocity.Y = obj.state(5);
            obj.parameters.BodyXYZVelocity.Z = obj.state(6);
            obj.parameters.BodyEulerAngle.Phi = obj.state(7);
            obj.parameters.BodyEulerAngle.Theta = obj.state(8);
            obj.parameters.BodyEulerAngle.Psi = obj.state(9);
            obj.parameters.BodyAngularRate.dPhi = obj.state(10);
            obj.parameters.BodyAngularRate.dTheta = obj.state(11);
            obj.parameters.BodyAngularRate.dPsi = obj.state(12);
        end
        
        % Set attitide control action for current step of simulation
        function obj = AttitudeControlAction(obj, uMoment1, uMoment2, uMoment3)
            obj.Moments = [uMoment1, uMoment2, uMoment3]';
        end

        % Set total thrust control action for current step of simulation
        function obj = TotalThrustControlAction(obj, uTotalThrust) 
            obj.TotalThrust = uTotalThrust;
        end
    end
end