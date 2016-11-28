%% Basic first model of locust movement
% Constraints:
%- constant speed
%- few agents
%- periodic boundary conditions
%- No "faulty velocity/position perception" of the locusts.

clear all
clc

%Parameters
N = 3;          %nbr agents
s = 2;          %speed of agents
gSize = 10;     % grid size
timesteps = 1;

% Variables defined from parameters
nbrGridPos = gSize*gSize;
agentAcc = zeros(2, N);
grid = zeros(gSize, gSize);

%------------ initialization ------------
% Random agent initial values
agentPos = ceil(rand(1,N)*nbrGridPos);
grid(agentPos) = 1;
angles = rand(1,N)*360;
agentVel = s*[cos(angles); sin(angles)];

for i_time = 1:timesteps
    %Update position with new velocity
%     [b_x, b_y] = find(grid == 2);
%     newBurningTrees = [b_x b_y+1; b_x+1 b_y; b_x b_y-1; b_x-1 b_y];                 % index for top, right, down, left of burning tree
%     newBurningTrees = mod(newBurningTrees-1, N) + 1;                                % periodic boundary conditions
%     newBurningTrees = sub2ind([N N], newBurningTrees(:,1), newBurningTrees(:,2));   % convert to linear indices for convenience
%     newBurningTrees = newBurningTrees( grid(newBurningTrees) == 1 );                % check what areas have trees in them that can catch fire
                
    %Plot agents with vectors
    [x, y] = find(grid == 1);
    [X,Y] = meshgrid(x,y);
    vecX = zeros(N, N);
    vecY = zeros(N, N);
    vecX(agentPos) = agentVel(1,:);
    vecY(agentPos) = agentVel(2,:);
    quiver(X, Y, vecX, vecY)
    NOT WORKING!!!
    
end