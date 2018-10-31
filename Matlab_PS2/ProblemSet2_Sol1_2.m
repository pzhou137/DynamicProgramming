% ProblemSet2_Sol1_2.m
%
% Matlab Script that solves Problem 1.2 of Problem Set 2 by applying value
% iteration.
% 
% Problem is taken from the book "Dynamic Programming and Optimal
% Control", Vol. 1, by D. Bertsekas. (Page 445, Problem 7.1b)
%
% Dynamic Programming and Optimal Control
% Fall 2017
% Problem Set 2, Problem 1.2
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Angela Schöllig
% aschoellig@ethz.ch
%
% --
% Revision history
% [12.11.09, AS] first version
% [21.10.12, RR] some restructuring
%
%

% clear workspace and command window
clear;
clc;

%% PARAMETERS
% Landing probability
p(2) = 0.95; % Slow serve

% Winning probability
q(1) = 0.6; % Fast serve
q(2) = 0.4; % Slow serve

% Define value iteration error bound
err = 1e-100;

% Define vector of incremental values for p(1)
p_incr_vec = 0 : 0.05 : 1;
prob_win_vec = zeros(1, length(p_incr_vec)); % probability of winning

%% VALUE ITERATION
for p_incr = p_incr_vec
    
    %% PARAMETER
    % Landing probability
	p(1) = p_incr;
    
    %% INITIALIZE PROBLEM
    % Our state space is S={0,1,2,3}x{0,1,2,3}x{1,2},
    % i.e. x_k = [score player 1, score player 2, serve]
    % ATTENTION: indices are shifted in code
    
    % Initialize costs to 1 (although any value would do)
    J = ones(4, 4, 2);
    
    % Initialize the optimal control policy: 1 represents Fast serve, 2
    % represents Slow serve
    FVal = ones(4, 4, 2);
    
    % Initialize cost-to-go
    costToGo = zeros(4, 4, 2);
    
    % Iterate until cost has converged
    iter = 0;
    while (1)
        
        % Increase counter
        iter = iter + 1;
        
        % Update the value
        for i = 1:3
            [costToGo(4,i,1),FVal(4,i,1)] = ...
                max(q.*p + (1-q).*p.*J(4,i+1,1)+(1-p).*J(4,i,2));
            [costToGo(4,i,2),FVal(4,i,2)] = ...
                max(q.*p + (1-q.*p).*J(4,i+1,1));
            [costToGo(i,4,1),FVal(i,4,1)] = ...
                max(q.*p.*J(i+1,4,1) + (1-p).*J(i,4,2));
            [costToGo(i,4,2),FVal(i,4,2)] = ...
                max(q.*p.*J(i+1,4,1));
            for j = 1:3
                [costToGo(i,j,1),FVal(i,j,1)] = ...
                    max(q.*p.*J(i+1,j,1) + (1-q).*p.*J(i,j+1,1)+(1-p).*J(i,j,2));
                [costToGo(i,j,2),FVal(i,j,2)] = ...
                    max(q.*p.*J(i+1,j,1) + (1-q.*p).*J(i,j+1,1));
            end
        end
        [costToGo(4,4,1),FVal(4,4,1)] = ...
            max(q.*p.*J(4,3,1) + (1-q).*p.*J(3,4,1)+(1-p).*J(4,4,2));
        [costToGo(4,4,2),FVal(4,4,2)] = ...
            max(q.*p.*J(4,3,1) + (1-q.*p).*J(3,4,1));
        
        % Check if cost has converged
        if (max(max(max(abs(J-costToGo))))/max(max(max(abs(costToGo)))) < err)
            % update cost and break
            J = costToGo;
            break;
        else
            % update cost
            J = costToGo;
        end
    end
    
    % Probability of player 1 winning the game
    prob_win_vec(p_incr == p_incr_vec)=J(1, 1, 1);
    
    % Display
    disp(['Terminated after ',num2str(iter,'%9.0f'),' iterations:',...
        ' For p_F = ',num2str(p_incr,'%9.2f'),...
        ', probability of winning is ',num2str(J(1, 1, 1),'%9.2f')]);
    
end

%% PLOT RESULT
figure(1);
plot(p_incr_vec, prob_win_vec,'*-');
title('Probability of the server winning a game');
xlabel('p_F'); ylabel('Probability of winning');

