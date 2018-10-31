% ProblemSet2_Sol2_3.m
%
% Matlab Script that solves Problem 2.3 of Problem Set 2 by applying value
% iteration.
%
% Problem is taken from the book "Dynamic Programming and Optimal
% Control", Vol. 1, by D. Bertsekas. (Page 446, Problem 7.3c)
%
% Dynamic Programming and Optimal Control
% Fall 2017
% Problem Set 2, Problem 2.3
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Sebastian Trimpe
% strimpe@ethz.ch
%
% --
% Revision history
% [11.11.2009, ST]    first version
% [06.11.2014, RR]    remove error bounds (not covered in lecture)
%


% clear workspace and command window
clear;
clc;


%% Program control
NUM_ITER = 1000;


%% Setup problem data
% Stage costs g.  g is a 2x2 matrix where the first index correponds to the
% state i=1,2 and the second index correponds to the admissible control
% input u=1 (=advertising/do research), or 2 (=don't advertise/don't do 
% research).
g = [4 6; -5 -3];

% Transition probabilities.  P is a 2x2x2 matrix, where the first index
% corresponds to the origin state, the second index corresponds to the
% destination state and the third input corresponds to the applied control
% input where (1,2) maps to (advertise/do research, don't advertise/don't 
% do research).  For example, the probability of transition from node 1 to 
% node 2 given that we do not advertice is P(1,2,2).
P = zeros(2,2,2);

% Advertise/do research (u=1):
P(:,:,1) = [0.8 0.2; 0.7 0.3];

% Don't advertise/don't do research (u=2):
P(:,:,2) = [0.5 0.5; 0.4 0.6];

% Discount factor.
%alpha = 0.9;
alpha = 0.99;


%% Value Iteration
% Initialize variables that we update during value iteration.
% Cost (here it really is the reward):
costJ = [0,0];
costJnew = [0,0];

% Policy
policy = [0,0];    

% Loop over value iterations k.
for k=1:NUM_ITER
    
    for i=1:2   % loop over two states
        % One value iteration step for each state.
        [costJnew(i),policy(i)] = max( g(i,:) + alpha*costJ*squeeze(P(i,:,:)) );
    end;
    
    % Save results for plotting later.
    res_J(k,:) = costJnew;

    % Construct string to be displayed later:
    dispstr = ['k=',num2str(k,'%5d'), ...
        '   J(1)=',num2str(costJnew(1),'%6.4f'), ...
        '   J(2)=',num2str(costJnew(2),'%6.4f'), ...
        '   mu(1)=',num2str(policy(1),'%3d'), ...
        '   mu(2)=',num2str(policy(2),'%3d')];
    
    % Update the cost.
    costJ = costJnew;
    
    % Display:
    disp(dispstr);

end;


% display the optained costs
disp('Result:');
disp(['  J*(1) = ',num2str(costJ(1),'%9.6f')]);
disp(['  J*(2) = ',num2str(costJ(2),'%9.6f')]);
disp(['  mu*(1) = ',num2str(policy(1))]);
disp(['  mu*(2) = ',num2str(policy(2))]);



%% Plots
% Plot costs over iterations.
kk = 1:size(res_J,1);

figure;
subplot(2,1,1);
plot(kk,[res_J(:,1)],[kk(1),kk(end)],[costJ(1), costJ(1)],'k');
grid;
legend('J_k(1)','J*(1)');
xlabel('iteration');

subplot(2,1,2);
plot(kk,[res_J(:,2)],[kk(1),kk(end)],[costJ(2), costJ(2)],'k');
grid;
legend('J_k(2)','J*(2)');
xlabel('iteration');


