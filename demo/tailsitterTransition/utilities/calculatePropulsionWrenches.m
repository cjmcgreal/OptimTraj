function [forces, moments] = calculatePropulsionWrenches(u, p) 
%
% Calculates forces and moments, from a propulsion system, for a given
% control vector.
%
% Inputs:
%   u = [n x n_time] control vector (i.e. throttle; typically 0 < u < 1)
%                       n = number of motors
%                       n_time = number of time steps
%   p = [struct] plant model struct with required fields:
%       .propulsion = [struct][n x 1] see definePropulsionModel.m
%       .environ.rho = [scalar] air density [kg/m^3]
%
% Outputs:
%   forces = [3 x n_time x n_motors]
%   moments = [n_time x 3 x n_motors]
%
% Written by Conrad McGreal 2020-02-27

n_motors = numel(p.propulsion) ; 
n_time = size(u,2) ; 

forces = zeros(3,n_time,n_motors) ; 
moments = zeros(3,n_time,n_motors) ; 

for i=1:n_motors % actuators
    this_motor = p.propulsion(i) ; 
    this_RPM = u(i,:) * this_motor.maxRPM ; 

    % Compute wrenches on prop (prop frame)
    [thrust, torque] = computePropOpPoint(this_RPM, p.environ.rho, this_motor.d_prop, ...
        this_motor.C_t, this_motor.C_q) ; 

    % Compute prop wrenches in body frame
    this_force = this_motor.thrustAxis' * thrust  ; 

    % Compute moments due to thrust 
    thrustArm = this_motor.thrustLocation - p.inertial.cg ;  % moment arm of motor to CG
    thrustArms = repmat(thrustArm,size(this_force,2),1) ; 
    thrustMoments = cross(thrustArms,this_force')  ; % moment, due to thrust, on vehicle

    % Reverse torque direction if required.
    if this_motor.isSpinDirectionCCW == 1 
       torqueAxis = -this_motor.thrustAxis ; 
    else
       torqueAxis = this_motor.thrustAxis ; 
    end

    % Compute moments due to countertorque. 
    torMoments = torque' * torqueAxis ; 

    % Add moments due to countertorque and moments due to thrust
    this_moment = torMoments + thrustMoments ; 
    this_moment = this_moment'; 
    
    % save
    forces(:,:,i) = this_force; 
    moments(:,:,i) = this_moment; 
end
