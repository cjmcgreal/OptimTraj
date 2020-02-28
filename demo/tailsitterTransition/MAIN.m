% MAIN - Minimum Impulse Boundary Value Problem
%
% Solve a minimum-impulse boundary value problem for a 3D (6 DOF) quadcopter with limits on the state and control. 
%
% The control is the throttle, u, which acts as normalized RPM, where 0 < u < 1 and 0 < RPM < maxRPM for each motor.
%
% This example includes simple planar aerodynamics.

% Define a plant model
[p] = definePlantModel() ;  % factored into it's own (input-less) function for code readability

% Trajectory Parameters:
uMax = 1 ; % maximum control input; note: when u(i) = 1 => motor RPM = p.propulsion(i).maxRPM

%% Boundary value problem setup
% initial boundary
problem.bounds.initialTime.low = 0; % bounds on initial time
problem.bounds.initialTime.upp = 0;

initialState = zeros(12,1); % initialize initial state vector
problem.bounds.initialState.low = initialState;
problem.bounds.initialState.upp = initialState;

% final boundary 
problem.bounds.finalTime.low = 1; % bounds on final time
problem.bounds.finalTime.upp = 10;

finalState = zeros(12,1);   % initialize
finalState(1) = 10; % assign non-zero state values.

problem.bounds.finalState.low = finalState;
problem.bounds.finalState.upp = finalState;

%% State and control bounds
problem.bounds.state.low = -100*ones(size(initialState)); % State
problem.bounds.state.upp = 100*ones(size(initialState)); 

problem.bounds.control.low = [0;0;0;0] ; % Control
problem.bounds.control.upp = [uMax;uMax;uMax;uMax] ;   

%% Guess at the initial trajectory
problem.guess.time = [0,5];
problem.guess.state = [initialState, finalState];
problem.guess.control = ones(4,2);

%% Solution
% User-defined dynamics and objective functions
problem.func.dynamics = @(t,x,u)( dynAircraft3d(x,u,p) );
problem.func.bndObj = @(t0,x0,tF,xF)( tF - t0 ); % minimum time  -- primary objective
problem.func.pathObj = @(t,x,u)( sum(0.001*u.^2) ); %minimum jerk  -- regularization

% Select a solver:
problem.options(1).method = 'trapezoid';
problem.options(1).trapezoid.nGrid = 8;
problem.options(2).method = 'trapezoid';
problem.options(2).trapezoid.nGrid = 16;

% Example syntax to run 'hermiteSimpson' solver.  Can take a while to run:  
% problem.options(3).method = 'hermiteSimpson';
% problem.options(3).hermiteSimpson.nSegment = 15;

% Solve the problem
soln = optimTraj(problem);

%% Postprocessing
% unpack solution
t = soln(end).grid.time;
q = soln(end).grid.state(1,:);
dq = soln(end).grid.state(2,:);
ddq = soln(end).grid.state(3,:);
u = soln(end).grid.control ;

% Plot the solution:
% plotQuadRotor3d(soln)