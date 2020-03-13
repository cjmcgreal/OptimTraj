% MAIN - Minimum Time Boundary Value Problem
%
% Solve a minimum-time boundary value problem for a 3D (6 DOF) quadcopter with limits on the state and control. 
%
% The control is the throttle, u, which acts as normalized RPM, where 0 < u < 1 and 0 < RPM < maxRPM for each motor.
% 

clc; clear;
addpath ../../ ./utilities ./test

%% Requirement specifics (GNC001)
% Sustained headwind of 10 kts. => dX = 5 m/s => finalState(7) = 5
% Altitude held => Z = 0 => initialState(3) = finalState(3) = 0. => Problem.bounds.state.low(3) = -1 ; Problem.bounds.state.high(3) = 1 ; 
% Note: no bounds on final X position => finalState.low(1) = 0 ; finalState.high(1) = 100 

%% Load plant model 
plotflag = 1 ; 
[p] = definePlantModelSwift021SN018NoAero(plotflag) ; 

%% Trajectory Parameters:
uMax = 1 ; % maximum control input; when u = 1; RPM = maxRPM.

%% Boundary value problem:
%% Initial state
initialState = zeros(12,1) ; % initialize 6 DOF with speeds (XYZ, RPY, dXYZ, dRPY)

problem.bounds.initialState.low = initialState;
problem.bounds.initialState.upp = initialState;

problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;

% wide limits
problem.bounds.initialState.low(4) = -pi/4 ; % wide limits on final MC roll attitude
problem.bounds.initialState.upp(4) = pi/4 ; % wide limits on final MC roll attitude
problem.bounds.initialState.low(5) = -pi/4 ; % wide limits on final pitch attitude
problem.bounds.initialState.upp(5) = pi/4 ; % wide limits on final pitch attitude
problem.bounds.initialState.low(6) = -pi/4 ; % wide limits on final yaw attitude
problem.bounds.initialState.upp(6) = pi/4 ; % wide limits on final yaw attitude


%% Final state
% For this requirement (SSGNC001) what's important is that the speed and
% altitude are correct (i.e. elements dX = 10
finalState = zeros(12,1) ;   % initialize

% assign non-zero final state values.
finalState(7) = 5 ; % forward velocity; 

problem.bounds.finalState.low = finalState;
problem.bounds.finalState.upp = finalState;

% wide limits
problem.bounds.finalState.low(1) = 0 ; % wide limits on final x position
problem.bounds.finalState.upp(1) = 100 ; % wide limits on final x position
problem.bounds.finalState.low(2) = 0 ; % wide limits on final y position
problem.bounds.finalState.upp(2) = 100 ; % wide limits on final y position
problem.bounds.finalState.low(3) = 0 ; % wide limits on final z position
problem.bounds.finalState.upp(3) = 100 ; % wide limits on final z position

problem.bounds.finalState.low(4) = -pi/4 ; % wide limits on final MC roll attitude
problem.bounds.finalState.upp(4) = pi/4 ; % wide limits on final MC roll attitude
problem.bounds.finalState.low(5) = -pi/4 ; % wide limits on final pitch attitude
problem.bounds.finalState.upp(5) = pi/4 ; % wide limits on final pitch attitude
problem.bounds.finalState.low(6) = -pi/4 ; % wide limits on final MC yaw attitude
problem.bounds.finalState.upp(6) = pi/4 ; % wide limits on final MC yaw attitude


problem.bounds.finalTime.low = 0.1;
problem.bounds.finalTime.upp = 100;

%% Control bounds
problem.bounds.control.low = [0;0;0;0] ;
problem.bounds.control.upp = [uMax;uMax;uMax;uMax] ;   

%% State bounds
problem.bounds.state.low = -100*ones(size(initialState)) ;
problem.bounds.state.upp = 100*ones(size(initialState)) ; 
% problem.bounds.state.low(3) = -1 ; % altitude; meters
% problem.bounds.state.upp(3) = 1 ;  % altitude; meters

% Guess at the initial trajectory
problem.guess.time = [0,5];
problem.guess.state = [initialState, finalState];
problem.guess.control = ones(4,2);

%% Solver 
% User-defined dynamics and objective functions
problem.func.dynamics = @(t,x,u)( dynQuadRotor3d(x,u,p) );
problem.func.bndObj = @(t0,x0,tF,xF)( tF - t0 ); % minimum time  -- primary objective
% problem.func.pathObj = @(t,x,u)( sum(0.001*u.^2) ); %minimum jerk  -- regularization

% Select a solver:
% problem.options(1).method = 'trapezoid';
% problem.options(1).trapezoid.nGrid = 8;
problem.options(1).method = 'trapezoid';
problem.options(1).trapezoid.nGrid = 16;
% problem.options(3).method = 'hermiteSimpson';
% problem.options(3).hermiteSimpson.nSegment = 15;

%% Solve the problem
soln = optimTraj(problem);
t = soln(end).grid.time;
q = soln(end).grid.state(1,:);
dq = soln(end).grid.state(2,:);
ddq = soln(end).grid.state(3,:);
u = soln(end).grid.control ;

%% Plot the solution:
plotQuadRotor3d(soln)



