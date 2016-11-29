%% Basic first model of locust movement
% Constraints:
%- constant speed
%- few agents
%- periodic boundary conditions
%- No "faulty velocity/position perception" of the locusts.

clear all
clc

%Parameters
N = 10;          %nbr agents
s = 2;          %speed of agents
gSize = 10;     % grid size
timesteps = 1;  % how many timesteps to take
dt = 0.5;         % time step (how far the agents will move at each step)
W_a = 1;
W_m = 1;
W_r = 2;
sightRadius = 4;
repulsionRadius = 2;

% Variables defined from parameters
nbrGridPos = gSize*gSize;
agentAcc = zeros(2, N);
grid = zeros(gSize, gSize);

%------------ initialization ------------
% Random agent initial values
agentPos = ceil(rand(1,N)*nbrGridPos);  %can appear on same spot!!
% agentPos = [nbrGridPos/2 + 5, nbrGridPos/2 + 3];   %for testing with two
angles = rand(1,N)*360;
agentVel = s*[cos(angles); sin(angles)];

for i_time = 1:timesteps
    [x, y] = ind2sub([gSize,gSize],agentPos);
    %########## Calculate Forces
    for i = 1:N
       %get all agents to take in to account for agent i and save distance
       r = [x - x(i); y - y(i)];    %line betwen two agents
       r_dist = sqrt(sum(r.^2));    %distance between two agents
       
       agentsOfInterest = r_dist < sightRadius;         %save only agents that are close enough
       nbrInterestingAgents = sum(agentsOfInterest);
       r = r(:, agentsOfInterest);
       r_dist = r_dist(:, agentsOfInterest);
       v = [agentVel(1,i) - agentVel(1,agentsOfInterest); agentVel(2,i) - agentVel(2,agentsOfInterest)];
       
       relVel = zeros(1, nbrInterestingAgents);
       f_aANDm = zeros(2, nbrInterestingAgents);
       for j = 1:nbrInterestingAgents
           relVel(j) = v(:,j)'*r(:,j);
           f_aANDm(:,j) = relVel(j)./r_dist(j)*r(:, j);
       end
       f_aANDm(:, relVel > 0) = f_aANDm(:, relVel > 0)*W_m;     %moving away
       f_aANDm(:, relVel < 0) = f_aANDm(:, relVel < 0)*W_a;     %approaching
%      OBS!!!! Also need repulsive force but do that later once you know
%      that it works!!!!!!!!!!!!!!
       
    end
    
    %########## Update position with new velocity

    x = round(x + agentVel(1,:)*dt);
    y = round(y + agentVel(2,:)*dt);
    %take care of periodic boundary conditions
    x = mod(x-1, gSize) + 1;
    y = mod(y-1, gSize) + 1;

    
    agentPos = sub2ind([gSize,gSize], x, y);
    
    %Plot agents with vectors
    quiver(x,y,agentVel(1,:), agentVel(2,:), 0);
    axis([0 gSize 0 gSize]);
    drawnow
    pause(0.2)
end