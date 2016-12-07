%% Basic first model of locust movement
% Constraints:
%- constant speed
%- few agents
%- periodic boundary conditions
%- No "faulty velocity/position perception" of the locusts.

clear all;
clc;

%Parameters
global gSize sightRadius;
sightRadius = 10;   % how far the locusts has to be to interact with each other.
gSize = 20;         % grid size
N = 100;            % nbr agents
s = 2;              % speed of agents
timesteps = 100;    % how many timesteps to take
dt = 0.5;           % time step (how far the agents will move at each step)
W_a = -5;
W_m = 5;
W_r = 2;
repulsionRadius = 2;

% Variables defined from parameters
agentAcc = zeros(2, N);

%For testing (can remove later)
newAngles = zeros(1,N);
newAgentVel = zeros(2, N);
quitting = 0;
quitting2 = 0;
radiusPlot(1:N) = sightRadius;

% ------------ Initialization ------------
% Random agent initial values
x = rand(1,N)*gSize;
y = rand(1,N)*gSize;
angles = rand(1,N)*2*pi;                    % velocity direction
agentVel = s*[cos(angles); sin(angles)];    % initial velocity


for i_time = 1:timesteps
    
    %expands grid in order to use boundary conditions
    [x2, y2, ID2] = ExpandGridForBoundaryConditions(x, y);
    
    %########## Calculate Forces
    for i = 1:N
        %independently of agent position we need to get distance to all
        %others. In the IF-statement we calculate distances to agents
        %positioned due to boundary conditions.
        agentID = 1:N;
        % Get distance to other agents
        r = [x - x(i); y - y(i)];
        agentID(i) = [];            %remove comparison to it self
        r(:,i) = [];
        if (x(i)<sightRadius || x(i)>gSize-sightRadius || y(i)<sightRadius || y(i)>gSize-sightRadius)    %if we need to think about periodic boundary conditions
            agentID = [agentID, ID2];
            r = [r, [x2-x(i); y2-y(i)]];
        end
        
        r_dist = sqrt(sum(r.^2));                       %distance between two agents
        agentsOfInterest = r_dist < sightRadius;        %save only agents that are close enough
        agentID = agentID(agentsOfInterest);             %get list of the interesting agents
        nbrInterestingAgents = sum(agentsOfInterest);
        r = r(:, agentsOfInterest);
        r_dist = r_dist(:, agentsOfInterest);
        
        v = zeros(2,nbrInterestingAgents);
        for j = 1:nbrInterestingAgents
            v(:,j) = agentVel(:,i) - agentVel(:, agentID(j) );
        end
        
        relVel = zeros(1, nbrInterestingAgents);
        f_aANDm = zeros(2, nbrInterestingAgents);
        nbrInSightRadius = 0;
        nbrInrepellingRange = 0;
        for j = 1:nbrInterestingAgents
            relVel(j) = v(:,j)'*r(:,j)/r_dist(j);
            
            if( r_dist(j) ~= 0 && r_dist(j) > repulsionRadius)
                f_aANDm(:,j) = relVel(j)*r(:, j)./r_dist(j);
                nbrInSightRadius = nbrInSightRadius + 1;
            elseif(r_dist(j)~=0)
                f_r = r_dist(j);
                nbrInRepellingRange = 0;
            end
%             if(~sum( sum(isnan(f_aANDm),2) )==0 )
%                 'first if'
%                 quitting2 = 1;
%                 break;
%             end
        end
%         if( quitting2 )
%             quitting = 1;
%             break;
%         end
        f_r = f_r * -W_r*s/nbrInRepellingRange;

        f_aANDm(:, relVel > 0) = f_aANDm(:, relVel > 0)*W_m;     %moving away
        f_aANDm(:, relVel < 0) = f_aANDm(:, relVel < 0)*W_a;     %approaching
        f_aANDm = f_aANDm/nbrInSightRadius;
        
        f_theta = zeros(1, nbrInSightRadius + nbrInRepellingRange);
        for j = 1:nbrInSightRadius
            f_theta(j) = [-sin(angles(i)), cos(angles(i))]*f_aANDm(:,j);
        end
        for j = 1:nbrInRepellingRange
            f_theta(j + nbrInSightRadius) = [-sin(angles(i)), cos(angles(i))]*f_r(:,j);
        end
        
        %update velocity
        if( ~isempty(f_theta) )
            newAngles(i) = angles(i) + sum(f_theta)*dt;
        else
            newAngles(i) = angles(i);
        end
        newAgentVel(:,i) = s*[cos(newAngles(i)); sin(newAngles(i))];
        
        %      OBS!!!! Also need repulsive force but do that later once you know
        %      that it works!!!!!!!!!!!!!!
    end
%     if( quitting )
%         disp('error - exiting algorithm')
%         break
%     end
    
    %Plot new velocities to see effect
%     plot(x,y,'.')
%     quiver(x,y,newAgentVel(1,:), newAgentVel(2,:), 0, 'r');
%     axis([0 gSize 0 gSize]);
%     drawnow
    %     viscircles([x',y'], radiusPlot);
%     waitforbuttonpress
    
    agentVel = newAgentVel;
    angles = newAngles;
    %########## Update position with new velocity
    x = x + agentVel(1,:)*dt;
    y = y + agentVel(2,:)*dt;
    %take care of periodic boundary conditions
    x = mod(x-1, gSize) + 1;
    y = mod(y-1, gSize) + 1;
    
    %Plot agents with vectors
    hold off
%     quiver(x,y,agentVel(1,:), agentVel(2,:), 0);
    plot(x,y,'.')
    axis([0 gSize 0 gSize]);
    drawnow
end
