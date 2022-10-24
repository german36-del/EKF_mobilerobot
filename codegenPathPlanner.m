function path = codegenPathPlanner(map,startPose,goalPose)
% Copyright 2021 The MathWorks, Inc.
    %#codegen
    % Create a state space object
    stateSpace = stateSpaceSE2;
    
    % Create a state validator object
    validator = validatorOccupancyMap(stateSpace);

    % Create a binary occupancy map and assign the map to the state 
    % validator object.
    validator.Map = binaryOccupancyMap(map);
    
    % Set the validation distance for the validator
    validator.ValidationDistance = 1;
    
    % Assign the state validator object to the plannerHybridAStar object
    planner = plannerHybridAStar(validator);
    
    % Compute a path for the given start and goal poses
    pathObj = plan(planner,startPose,goalPose);
    
    % Extract the path poses from the path object
    path = pathObj.States;
end