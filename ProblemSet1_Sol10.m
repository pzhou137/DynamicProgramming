% ProblemSet1_Sol10.m
%
% Matlab Script that solves Problem 10 of Problem Set 1 by applying the
% DP algorithm.
%
% Dynamic Programming and Optimal Control
% Fall 2014
% Problem Set 1, Problem 10
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Robin Ritz
% rritz@ethz.ch
%
% --
% Revision history
% [20.09.2013, Robin Ritz]      first version
% [23.09.2013, Dario Brescianini]
% [05.10.2017, Weixuan Zhang] 
%

%% clear workspace and command window
clear all;
close all;
clc;

%% Initialize variables
p_w = 1/3; % probability that wk = 1
num_states = 11; % x = 0, ..., 10 -> there are 11 possible states
num_inputs = 11; % 0 <= xk + uk <= 10 -> there are always 11 possible inputs
num_timesteps = 10; % corresponds to N
x = 0 : num_states - 1; % state vector containing all possible states 0, ..., 10
J_opt = zeros( num_states, num_timesteps + 1 ); % matrix that will store the optimal cost-to-go for each possible state and for each time step
u_opt = zeros( num_states, num_timesteps ); % matrix that will store the optimal input for each possible state and for each time step ( except the last one )

%% Initialize DP algorithm
J_opt( : , end ) = x.^2; % this implements gN(xN) = xN^2

%% Perform DP algorithm
% Note that since Matlab matrices start at index 1, there is a shift when
% we use the index k ( that starts at 0 ) to access a matrix. For
% example, to access J_opt for time index k, we write J_opt( : , k + 1 ).
for k = ( num_timesteps - 1 ) : -1 : 0 % start at k = N-1 and decrease to k = 0
    
    % Iterate over all possible states
    for i = 1 : num_states
        
        % Read state
       	xk = x( i );
        
        % Determine set of allowed inputs
        set_Uk = -xk : 10 - xk; % this implements 0 <= xk + uk <= 10
        
        % Compute expected cost-to-go for each allowed input
        cost_to_go = zeros( 1, num_inputs );
        for m = 1 : num_inputs;
            
            % Read input
            uk = set_Uk( m );
           
            % Compute cost-to-go for this input
            x_ref = ( k - 5 )^2;
            cost_to_go( m ) = ( xk - x_ref )^2 + uk^2 ...
                + p_w * J_opt( ( xk + uk ) == x, k + 2 ) ...
                + ( 1 - p_w ) * J_opt( xk == x, k + 2 );
            
        end
        
        % Find minimal cost-to-go and corresponding optimal input
        [ J_opt( i, k + 1 ), index_u_opt ] = min( cost_to_go );
      	u_opt( i, k + 1 ) = set_Uk( index_u_opt );
        
    end
    
end

%% Print result
printmat( J_opt, 'optimal cost-to-go',...
    'x=0 x=1 x=2 x=3 x=4 x=5 x=6 x=7 x=8 x=9 x=10', ...
    'k=0 k=1 k=2 k=3 k=4 k=5 k=6 k=7 k=8 k=9 k=10' ); % printmat() requires control system toolbox
printmat( u_opt, 'optimal input', ...
    'x=0 x=1 x=2 x=3 x=4 x=5 x=6 x=7 x=8 x=9 x=10', ...
    'k=0 k=1 k=2 k=3 k=4 k=5 k=6 k=7 k=8 k=9' ); % printmat() requires control system toolbox

%% Visualize result
% This is an example of how one can visualize the optimal policy. We
% create a plot showing the state transitions for wk = 1. ( For
% wk = 0 the state does not change. )

% Select figure
figure( 1 );

% Plot all states xk for all timesteps k
k = 0 : num_timesteps;
[ p, q ] = meshgrid( k, x );
plot( p(:), q(:), 'ko', 'LineWidth', 2, 'MarkerSize', 10 ); hold on;

% Plot state transitions from xk to xk + uk
x_plus_u = x + u_opt( : , 1 )';
for i = 1 : num_timesteps
    kk = [ i - 1, i ];
    x_plus_u = x + u_opt( : , i )';
    state_trans_handle = plot( kk, [ x; x_plus_u ], '-b', 'LineWidth', 2 ); hold on;
end

% Plot x_ref( k )
x_ref_handle = plot( k, ( k - 5 ).^2, '-r', 'LineWidth', 2 ); hold off;

% Do some formatting and labeling
axis( [ -1, num_timesteps + 1, x( 1 ) - 1, x( end ) + 1 ] );
xlabel( 'k', 'Interpreter', 'tex' );
ylabel( 'x_k', 'Interpreter', 'tex' );
legend( [x_ref_handle, state_trans_handle( 1 ) ], 'x_{ref}(k)', 'state transitions for w_k = 1', 'Location', 'NorthOutside' );



