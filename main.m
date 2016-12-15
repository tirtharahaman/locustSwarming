%% Basic first model of locust movement
% Constraints:
%- constant speed
%- few agents
%- periodic boundary conditions
%- No "faulty velocity/position perception" of the locusts.

%Description:
%We want to make an evolutionary algorithm where we use fitness and
%generations to derive the optimal behaviour (W_a & W_m) of locusts.
%We can only get the optimal behaviour for a specific density, so in the
%end, we will have to run our model over many different densities.
%Right now W_a and W_m is set to constants but in the evolutionary
%algorithm me want each locust to be randomed initial W_a and W_m.

clear all;
clc;
global gSize sightRadius;   %make global so that functions do not need them as input.

%Parameters
timesteps = 1000000;        % how many timesteps to take; large fail-safe exit
repulsionRadius = 1;        % how close locusts has to be before repelling force sets in
s = repulsionRadius*15;     % speed of agents
sightRadius = 8*repulsionRadius;                            % how close the locusts has to be to interact with each other
N = 300;                    % nbr agents
density = 2;                %density
gSize = sightRadius*sqrt(N/density);                        % grid side length
tTransient = 100;           % transient time from starting conditions for fitness calculation
tFit = 500;                 % nbr of timesteps for fitness calculation
dt = 0.02;                  % time step
% W_a = ones(1,N)*-1;
% W_m = ones(1,N);
upperLimit = 1;             % Limit for W_a and W_m. Checks this after evolution...
lowerLimit = -1;            % ...so that W_a and W_m donot cross boundary
W_a = lowerLimit + rand(1, N) * (upperLimit-lowerLimit);    % reaction to approaching locusts
W_m = lowerLimit + rand(1, N) * (upperLimit-lowerLimit);    % reaction to moving away locusts
W_r = 2;                    % repelling force constant.
randDegree = 0.02;

meanW_a = mean(W_a);        % initial mean of W_a and W_m
meanW_m = mean(W_m);
lastMeanW_a = upperLimit;   % initialized to max value
lastMeanW_m = upperLimit;

c_r = 100;                  % cost of cannibalism at the rear
c_f = 10;                   % cost of cannibalism at the front
b = 20;                     % benefit of cannibalism
W_b = 0.2;                  % relative weight of benefits to costs (0.0-1.0)
sigma_mu = 0.01;            % strength of mutation
tolerance = 0.005;          % tolerance of W_a and W_m; to check when to stop simulation

cost = zeros(1, N);
benefit = zeros(1, N);
fitness = zeros(1, N);

% Variables defined from parameters
% agentAcc = zeros(2, N);
newAngles = zeros(1, N);
newAgentVel = zeros(2, N);

%For testing (can remove later)
% radiusPlot(1:N) = sightRadius;

% ------------ Initialization ------------
% Random agent initial values
x = rand(1,N)*gSize;
y = rand(1,N)*gSize;
angles = rand(1,N)*2*pi;                    % velocity direction
agentVel = s*[cos(angles); sin(angles)];    % initial velocity

transientFlag = true;                       % checks whether transient period is ON;...
transientPeriod = 0;                        % ...doesnot perform fitness calculation

fprintf('N(W_a)>0: %d, mean:%2.4f, N(W_m)>0: %d, mean:%2.4f\n', numel(find(W_a>0)), meanW_a, numel(find(W_m>0)), meanW_m);

%start: timeStep for-loop
for i_time = 1:timesteps

    %expands grid in order to use boundary conditions (see function
    %description for more detail)
    [x2, y2, ID2] = ExpandGridForBoundaryConditions(x, y);

    % This FOR-LOOP Calculates and updates Forces
    % start: agent for-loop
    for i = 1:N

        agentID = 1:N;                                  %used to get the velocity related to locusts later on.

        %Get all relative positions to locust i
        r = [x - x(i); y - y(i)];
        agentID(i) = [];                                %remove comparison to it self
        r(:,i) = [];
        
        %IF locust i is so close to the boundary that it's "sight" should
        %reach over the boundary we have to also take in to account the
        %expanded grid.
        if (x(i)<sightRadius || x(i)>gSize-sightRadius || y(i)<sightRadius || y(i)>gSize-sightRadius)    %if we need to think about periodic boundary conditions
            %x2,y2,ID2 comes from expanding the grid for taking the
            %boundary conditions in to account in an easy way.
            agentID = [agentID, ID2];
            r = [r, [x2-x(i); y2-y(i)]];
        end
        
        %get distance between locusts and filter out the, for locust i,
        %important other locusts
        r_dist = sqrt(sum(r.^2));                       %distance between two agents
        agentsOfInterest = r_dist < sightRadius;        %save only agents that are close enough
        agentID = agentID(agentsOfInterest);            %get list of the interesting agents
        nbrInterestingAgents = sum(agentsOfInterest);
        r = r(:, agentsOfInterest);
        r_dist = r_dist(:, agentsOfInterest);
        
        %Get relative velocity between locust i and the important locusts
        v = [agentVel(1, agentID) - agentVel(1,i); agentVel(2, agentID) - agentVel(2,i)];
        
        %in this FOR-LOOP calculate forces resulting from approaching and
        %moving awway locusts
        f_aANDm = zeros(2, nbrInterestingAgents);
        nbrInSightRadius = 0;
        nbrInrepellingRange = 0;
        relVel = sum(v'.*r',2)./r_dist';
        for j = 1:nbrInterestingAgents
            if( r_dist(j) ~= 0)
                f_aANDm(:,j) = relVel(j)*r(:, j)./r_dist(j);
            end
        end

        nApproaching = sum(relVel < 0);
        nMovingAway = sum(relVel > 0);
        if nApproaching > 0
            f_aANDm(:, relVel < 0) = f_aANDm(:, relVel < 0)*W_a(1, i)/nApproaching;     %approaching
        end
        if nMovingAway > 0
            f_aANDm(:, relVel > 0) = f_aANDm(:, relVel > 0)*W_m(1, i)/nMovingAway;     %moving away
        end


        %calculate forces resulting from locusts repelling force (force
        %because they are too close to each other).

        nbrInRepellingRange = sum(r_dist < repulsionRadius);
        if(nbrInRepellingRange ~= 0)
            f_r = sum(r(:,r_dist < repulsionRadius), 2);
            f_r = f_r * -W_r*s/nbrInRepellingRange;
        else
            f_r = [0;0];
        end

        %get forces in theta- (angle-) direction
        forceDirection = [-sin(angles(i)), cos(angles(i))];
        f_theta = sum(forceDirection*f_aANDm);                  %from approaching and moving away locusts
        f_theta = f_theta + forceDirection*f_r;                 %from repelling agents

        %calculate cost and benefit
        if transientFlag == true                                % when trasient period is ON, fitness params
          transientPeriod = transientPeriod + 1;                % are not calculated
          if mod(transientPeriod, tTransient) == 0
            transientFlag = false;                              % transient period is over after tTransient time
          end
        else                                                    % fitness period; time calculate fitness params
          agentsInRepulsionRadius = find(r_dist < repulsionRadius);
          for j = 1:length(agentsInRepulsionRadius)
              direction = sum((r(:, j)/r_dist(j)) .* v(:,j));
              cost(1, i) = cost(1, i) + (c_r * heaviside(-direction) + c_f * heaviside(direction));
              benefit(1, i) = benefit(1, i) + b * heaviside(direction);
          end
        end

        %update velocity
        if( ~isempty(f_theta) )
            newAngles(i) = angles(i) + f_theta*dt + (rand(1)*2 - 1)*randDegree;
        else
            newAngles(i) = angles(i)  + (rand(1)*2 - 1)*randDegree;
        end
        newAgentVel(:,i) = s*[cos(newAngles(i)); sin(newAngles(i))];
    end
    %end: agent for-loop

    %Update old values
    agentVel = newAgentVel;
    angles = newAngles;
    x = x + agentVel(1,:)*dt;
    y = y + agentVel(2,:)*dt;
    x = mod(x-1, gSize) + 1;                                   %take care of periodic boundary conditions
    y = mod(y-1, gSize) + 1;
    
%FOR TESTING - Plot new velocities to see effect
%     plot(x,y,'.')
%     quiver(x,y,newAgentVel(1,:), newAgentVel(2,:), 0, 'r');
%     axis([0 gSize 0 gSize]);
%     drawnow
%     viscircles([x',y'], radiusPlot);
%     waitforbuttonpress
    
    %Plot agents with vectors
    hold off
%     quiver(x,y,agentVel(1,:), agentVel(2,:), 0);
    plot(x,y,'.')
    hold on
    plot(x(1), y(1), 'r.')
    axis([0 gSize 0 gSize]);
    set(gca, 'XTick', []);
    set(gca, 'YTick', []);
    drawnow;

    %Evolutionary part (Fitness, Selection, Mutation, New Generation)      
    if mod(i_time, tTransient+tFit) == 0                    % new generation is calculated after 
                                                            % every (tTransient + tFit) time
        %fitness calculation
        fitness = (W_b * benefit) - ((1-W_b) * cost);

        %tournament selection
        newW_a = zeros(size(W_a));
        newW_m = zeros(size(W_m));
        selectionParameter = 0.8;
        for i = 1:N
            agent1 = 1 + fix(rand * N);
            agent2 = 1 + fix(rand * N);

            pDraw = rand;
            
            if pDraw < selectionParameter
                if fitness(agent1) > fitness(agent2)
                    newW_a(i) = W_a(agent1);
                    newW_m(i) = W_m(agent1);
                else
                    newW_a(i) = W_a(agent2);
                    newW_m(i) = W_m(agent2);
                end
            else
                if fitness(agent1) > fitness(agent2)
                    newW_a(i) = W_a(agent2);
                    newW_m(i) = W_m(agent2);
                else
                    newW_a(i) = W_a(agent1);
                    newW_m(i) = W_m(agent1);
                end
            end
        end
        
%         %roulette-wheel selection
%         cumulativeFitness = [];
%         cumulativeScore = 0;
%         %making roulette-wheel
%         for i = 1:length(fitness)
%             cumulativeScore = cumulativeScore + fitness(1, i);
%             cumulativeFitness = [cumulativeFitness, cumulativeScore];
%         end
% 
%         %selection
%         newW_a = zeros(size(W_a));
%         newW_m = zeros(size(W_m));
%         for i = 1:N
%             pDraw = rand * cumulativeScore;
% 
%             for j = 1:N
%                 if pDraw < cumulativeFitness(1, j)
%                     newW_a(1, i) = W_a(1, i);
%                     newW_m(1, i) = W_m(1, i);
%                 end
%             end
%         end

        %mutation
        for i = 1:N
            mutation = -sigma_mu + rand * 2*sigma_mu;
            newW_a(1, i) = newW_a(1, i) + mutation;
            newW_m(1, i) = newW_m(1, i) + mutation;

            %traits are bounded by upper and lower limit (-5,5)
            if norm(newW_a(1, i)) > upperLimit
                newW_a(1, i) = sign(newW_a(1, i)) * upperLimit;
            end
            if norm(newW_m(1, i)) > upperLimit
                newW_m(1, i) = sign(newW_m(1, i)) * upperLimit;
            end
        end

        %new generation
        W_a = newW_a;
        W_m = newW_m;

        transientFlag = true;                       % transient period starts for new generation
        transientPeriod = 0;

        meanW_a = mean(W_a);
        meanW_m = mean(W_m);

        if norm(lastMeanW_a - meanW_a) < tolerance && norm(lastMeanW_m - meanW_m) < tolerance
          fprintf('N(W_a)>0: %d, mean:%2.4f, N(W_m)>0: %d, mean:%2.4f\n', numel(find(W_a>0)), meanW_a, numel(find(W_m>0)), meanW_m);
          break;                                        % converged to stable values for evolutionary traits
        else
          lastMeanW_a = meanW_a;
          lastMeanW_m = meanW_m;
        end
        
        fprintf('N(W_a)>0: %d, mean:%2.4f, N(W_m)>0: %d, mean:%2.4f\n', numel(find(W_a>0)), meanW_a, numel(find(W_m>0)), meanW_m);
    end
    %end: Evolutionary part

end
%end: timeStep for-loop
