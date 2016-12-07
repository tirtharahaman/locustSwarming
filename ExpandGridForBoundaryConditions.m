function [x2, y2, ID2  ] = ExpandGridForBoundaryConditions( x, y)
% ExpandGridForBoundaryConditions takes initial grid with positions and
% expands it (add on edges) as much as the agent sight is.
% EXAMPLE - gridSize = 10x10, sightRadius=2, if there is an agent at
% position (1,3) a new agent will be put at pos (9

% Outputs
%x2, y2 contains positions of agents within the expanded grid edges.
%ID2 contains the ID of each coordinate so that the right velocity can be
%used later on.

% Inputs:
%x = x position of agents
%y = y position of agents
%gSize = gridSize. Needed to determine how to get the boundary conditions.
%OBS variables sightRadius & gridSize must be global constants
global gSize sightRadius;
N = length(x);

agentID = 1:N;
agentsToMoveRight = x<sightRadius;
agentsToMoveUp    = y<sightRadius;
agentsToMoveLeft = x>gSize-sightRadius;
agentsToMoveDown = y>gSize-sightRadius;

agentsToMoveRightAndUp = agentsToMoveRight + agentsToMoveUp == 2;
agentsToMoveRightAndDown = agentsToMoveRight + agentsToMoveDown == 2;
agentsToMoveLeftAndUp = agentsToMoveLeft + agentsToMoveUp == 2;
agentsToMoveLeftAndDown = agentsToMoveLeft + agentsToMoveDown == 2;

%Move Right
posMoveRight_x = x(agentsToMoveRight) + gSize;
posMoveRight_y = y(agentsToMoveRight);
idToMoveRight = agentID(agentsToMoveRight);
%Move Left
posMoveLeft_x = x(agentsToMoveLeft) - gSize;
posMoveLeft_y = y(agentsToMoveLeft);
idToMoveLeft = agentID(agentsToMoveLeft);
%Move up
posMoveUp_x = x(agentsToMoveUp);
posMoveUp_y = y(agentsToMoveUp) + gSize;
idToMoveUp = agentID(agentsToMoveUp);
%Move Down
posMoveDown_x = x(agentsToMoveDown);
posMoveDown_y = y(agentsToMoveDown) - gSize;
idToMoveDown = agentID(agentsToMoveDown);
%Move Right and up (agents that are close to bottom left corner)
posMoveRightAndUp_x = x(agentsToMoveRightAndUp) + gSize;
posMoveRightAndUp_y = y(agentsToMoveRightAndUp) + gSize;
idToMoveRightAndUp = agentID(agentsToMoveRightAndUp);
%Move Right and down (agents that are close to top left corner)
posMoveRightAndDown_x = x(agentsToMoveRightAndDown) + gSize;
posMoveRightAndDown_y = y(agentsToMoveRightAndDown) - gSize;
idToMoveRightAndDown = agentID(agentsToMoveRightAndDown);
%Move left and top (agents that are close to top left corner)
posMoveLeftAndTop_x = x(agentsToMoveLeftAndUp) - gSize;
posMoveLeftAndTop_y = y(agentsToMoveLeftAndUp) + gSize;
idToMoveLeftAndTop = agentID(agentsToMoveLeftAndUp);
%Move left and down (agents that are close to top left corner)
posMoveLeftAndDown_x = x(agentsToMoveLeftAndDown) - gSize;
posMoveLeftAndDown_y = y(agentsToMoveLeftAndDown) - gSize;
idToMoveLeftAndDown = agentID(agentsToMoveLeftAndDown);

x2 = [posMoveDown_x, posMoveLeftAndDown_x, posMoveLeftAndTop_x, posMoveLeft_x, posMoveRightAndDown_x,... 
    posMoveRightAndUp_x, posMoveRight_x, posMoveUp_x];
y2 = [posMoveDown_y, posMoveLeftAndDown_y, posMoveLeftAndTop_y, posMoveLeft_y, posMoveRightAndDown_y,...
    posMoveRightAndUp_y, posMoveRight_y, posMoveUp_y];
ID2 = [idToMoveDown, idToMoveLeftAndDown, idToMoveLeftAndTop, idToMoveLeft, idToMoveRightAndDown,...
    idToMoveRightAndUp, idToMoveRight, idToMoveUp];


end

