function [notSuccesfullyPassed, currentWaypoint] = CheckWayPointTrack(bodyXYZPosition, actualTime, timeForWaypointPasage, wayPoints, positionTolerance,currentWaypoint)
    % Function that determines whether the quadcopter passes specified waypoints at given times within tolerance
    
    positionVector = [bodyXYZPosition.X bodyXYZPosition.Y bodyXYZPosition.Z];
    max_distance=max(abs(positionVector - wayPoints(currentWaypoint,:)));
    notSuccesfullyPassed = false;

    if max_distance<=positionTolerance
        currentWaypoint = currentWaypoint + 1;
        if currentWaypoint == length(timeForWaypointPasage)+1
            msgbox("Drone landed on the Finish line!!!\n simulation ended");
            notSuccesfullyPassed = true; % ending the simulation
            return;
        end
    end
    
    if actualTime>=timeForWaypointPasage(currentWaypoint)
        notSuccesfullyPassed = true;
        msgbox(['Quadcopter did not pass waypoint ' num2str(currentWaypoint) ...
                '. Actual Position: [' num2str(positionVector) ']. ' ...
                'Desired Position: [' num2str(wayPoints(currentWaypoint,:)) '].'], 'Error', 'error'); 
        return;
    end
       
    
    
end