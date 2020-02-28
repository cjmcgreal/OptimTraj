function [out] = dynBodyFrame(z_body, u, p)
%
% Computes combined (from all motors and props) forces and moments, and resultant acceleration, on a vehicle.
% Includes a simple aerodynamic model. 
% Does not include effects from gravity.
%
% INPUTS: 
%   z_body = [9,n] abbreviated state vector consists of [att; linVel;omega] where
%       att = [pitch; roll; yaw][rad]
%       linVel = [dx; dy; dz] [m/s] linear velocity
%       omega = [dpitch; droll; dyaw][rad/s] angular velocity.  
%   u = [N,n] (0-1) = control vector (i.e. "throttle") where:.  
%                       All elements should be: 0 < u(i) < 1 
%                       N = number of motors
%                       n = number of time steps
%   p = parameter struct: 
%       .inertial = [struct] with fields:
%           .m = [scalar] (kg) aircraft total mass 
%           .cg = [1,3] (m) center-of-gravity location in body coords.
%           .I = [3x3] (kg-m^2) inertia matrix kg-m^2]
%       .propulsion(i) = [struct] propulsion parameter struct with fields
%           .thrustAxis = [3x1] (unit vector) in body coordinates
%           .thrustlocation = [3x1] (m) XYZ in body coordinates.
%           .isSpinDirectionCCW = [bool] if true, propeller spin direction 
%                                    is counterclockwise around thrust axis.
%           .C_t = [scalar] () thrust coefficient see https://web.mit.edu/16.unified/www/FALL/thermodynamics/notes/node86.html
%           .C_q = [scalar] () torque coefficient
%           .maxRPM = [scalar] (RPM) maximum propeller RPM 
%           .maxTorque = [scalar] (Nm) maximum propeller torque
%           .d_prop = [scalar] (m) propeller diameter 
%       .aero = [struct] with fields:
%           .area = [m^2] aerodynamic reference area
%           .mac = [m] mean aerodynamic chord
%           .LUT = aerodynamic coefficient lookup table (see
%                   readme_aeroModel for details)
%       .environ = [struct] with fields:
%           .g = [scalar] (m/s/s) acceleration due to gravity
%           .rho = [scalar] (kg/m^3) air density
%
% OUTPUTS:
%   out = [6, n] = [linearAccelerations; angularAccelerations] where
%               linearAccelerations = [x_accel; y_accel; z_accel] (m/s/s) forces due to thrust
%               angularAccelerations = [pitch_accel; roll_accel; yaw_accel] (rad/s/s) moment due to countertorque.
%
% Written by Conrad McGreal 2020/01/27 

%% Unpack input state
eul = z_body(1:3,:);    % aircraft attitude
linVel = z_body(4:6,:); % linear world velocity
omega = z_body(7:9,:);  % attitude rates

%% Aero forces and moments
[aeroForces,aeroMoments] = calculateAeroWrenches(z_body, p.aero, p.environ.rho) ; 

%% Propulsive Wrenches
[propulsionForces, propulsionMoments] = calculatePropulsionWrenches(u, p);
totalPropulsionForce = sum(propulsionForces,3) ; % add together, for each time step, total forces from all motors.
totalPropulsionMoment = sum(propulsionMoments,3) ; % add together, for each time step, total moment from all motors.

%% Rigid body dynamics
% linear accelerations
resultantForce = totalPropulsionForce + aeroForces' ; 
linAccel = resultantForce./p.inertial.m ; % acceleration in each linear direction for every timestep

% angular accelerations
resultantMoment = totalPropulsionMoment + aeroMoments' ; 
bias = cross(omega,p.inertial.I*omega); % Eulerian bias acceleration term
angAccel = p.inertial.I\(resultantMoment-bias) ; % solve Euler's equation for ang eccleration

% Prepare output
out = [linAccel ; angAccel] ; 
