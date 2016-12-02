%% Basic first model of locust movement
% Constraints:
%- constant speed
%- few agents
%- periodic boundary conditions
%- No "faulty velocity/position perception" of the locusts.

clear all;
clc;

%Parameters
N = 2;              % nbr agents
s = 10;             % speed of agents
gSize = 20;         % grid size
timesteps = 100;    % how many timesteps to take
dt = 0.1;           % time step (how far the agents will move at each step)
W_a = -5;
W_m = 5;
W_r = 2;
sightRadius = 10;
repulsionRadius = 2;

% Variables defined from parameters
nbrGridPos = gSize*gSize;
agentAcc = zeros(2, N);
grid = zeros(gSize, gSize);

%For testing (can remove later)
newAngles = zeros(1,N);
newAgentVel = zeros(2, N);
quitting = 0;
quitting2 = 0;
radiusPlot(1:N) = sightRadius;

% ------------ Initialization ------------
% Random agent initial values
% agentPos = ceil(rand(1,N)*nbrGridPos);  %can appear on same spot!!
% We can get random init position the following way ensuring no two locusts are on the same spot
% x = randperm(gsize, numberOfLocusts);
% y = randperm(gsize, numberOfLocusts);
% agentPos = [x; y];    % Position vector
agentPos = [nbrGridPos/2 + 5, nbrGridPos/2 + 3];   %for testing with two
% Changing direction for 2 locusts works good. We should check if it is working for 3 locusts
% in the vicinity
angles = rand(1,N)*2*pi;
agentVel = s*[cos(angles); sin(angles)];

for i_time = 1:timesteps
    [x, y] = ind2sub([gSize,gSize],agentPos);   % Not required if randperm is used
    %########## Calculate Forces
    for i = 1:N
       %get all agents to take in to account for agent i and save distance
       agentID = 1:N;
       r = [x - x(i); y - y(i)];    %line betwen two agents
       
       agentID(i) = [];
       r(:,i) = [];                   %remove comparison to it self
       r_dist = sqrt(sum(r.^2));    %distance between two agents
       
       agentsOfInterest = r_dist < sightRadius;         %save only agents that are close enough
       agentID = agentID(agentsOfInterest)             %get list of the interesting agents
       nbrInterestingAgents = sum(agentsOfInterest);
       r = r(:, agentsOfInterest);
       r_dist = r_dist(:, agentsOfInterest);
       
       v = zeros(2,nbrInterestingAgents);
       for j = 1:nbrInterestingAgents
           v(:,j) = agentVel(:,i) - agentVel(:, agentID(j) );
       end
       v
       r
           
       
       relVel = zeros(1, nbrInterestingAgents);
       f_aANDm = zeros(2, nbrInterestingAgents);
       %IF WE GET TWO AGENTS ON SAME POSITION WE GET NaN. NEED TO TAKE CARE
       %OF THAT.
       for j = 1:nbrInterestingAgents
           relVel(j) = v(:,j)'*r(:,j)/r_dist(j)
           
           if( relVel(j) ~= 0)
               f_aANDm(:,j) = relVel(j)*r(:, j)./r_dist(j);
           end
           if(~sum( sum(isnan(f_aANDm),2) )==0 )
               'first if'
               quitting2 = 1;
               break;
           end
       end
       if( quitting2 )
           quitting = 1;
           break;
       end
       f_aANDm(:, relVel > 0) = f_aANDm(:, relVel > 0)*W_m;     %moving away
       f_aANDm(:, relVel < 0) = f_aANDm(:, relVel < 0)*W_a;     %approaching
       f_aANDm = f_aANDm/nbrInterestingAgents
        
       f_theta = zeros(1, nbrInterestingAgents);
       for j = 1:nbrInterestingAgents
           f_theta(j) = [-sin(angles(i)), cos(angles(i))]*f_aANDm(:,j)
       end
       %        f_theta = atan(f_aANDm(2,:)./f_aANDm(1,:));
       
       if( ~isempty(f_theta) )
           %update velocity
           newAngles(i) = angles(i) + sum(f_theta)*dt;
       else
           newAngles(i) = angles(i);
       end
       newAgentVel(:,i) = s*[cos(newAngles(i)); sin(newAngles(i))];
       
       %      OBS!!!! Also need repulsive force but do that later once you know
       %      that it works!!!!!!!!!!!!!!
    end
    if( quitting )
        disp('error - exiting algorithm')
        break
    end
    
    %Plot new velocities to see effect
    quiver(x,y,newAgentVel(1,:), newAgentVel(2,:), 0, 'r');
    axis([0 gSize 0 gSize]);
    drawnow
%     viscircles([x',y'], radiusPlot);
    waitforbuttonpress
    clc
    
    
    sum(sum(agentVel == newAgentVel,2))
    agentVel = newAgentVel;
    angles = newAngles;
    %########## Update position with new velocity
    x = round(x + agentVel(1,:)*dt);
    y = round(y + agentVel(2,:)*dt);
    %take care of periodic boundary conditions
    x = mod(x-1, gSize) + 1;
    y = mod(y-1, gSize) + 1;

    agentPos = sub2ind([gSize,gSize], x, y);    %might be unneccesary
 
    %Plot agents with vectors
    hold off
    quiver(x,y,agentVel(1,:), agentVel(2,:), 0);
    axis([0 gSize 0 gSize]);
    drawnow
    pause(0.4)
    hold on
end
