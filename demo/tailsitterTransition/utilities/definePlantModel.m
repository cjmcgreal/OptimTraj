function [p] = definePlantModel()
%
% Defines a plant model for 3D boundary value problem (with aerodynamics)
% 
% Factored out (from MAIN) for code readability. To make changes to plant model, make changes here directly.
%

p = struct() ; % init

% Inertial params
p.inertial.m = 5 ; 
p.inertial.I = [0.625 0 0; 0 0.625 0; 0 0 1.25] ; % inertia tensor coords: 
p.inertial.cg = [0 0 0] ; % (m) location of center of gravity

% Propulsion system params
% propulsion system parameters shared for all motors:
qRP.d_prop = 0.305*ones(4,1) ; % propeller diameter (m)
qRP.maxThrust = 25*ones(4,1) ; % thrust at 100% throttle (N)
qRP.maxRPM = 10000*ones(4,1) ; % RPM at 100% throttle (RPM)
qRP.maxTorque = ones(4,1) ;  % torque at 100% throttle (Nm)
qRP.thrustLocations = [0.5 0 0; 0 0.5 0; -0.5 0 0; 0 -0.5 0]; % motor locations (each row one motor in coords: [port, nose, top] 
qRP.thrustAxes = repmat([0 0 1],4,1) ; % thrust axes of each motor in coords port, nose, top.
qRP.isSpinDirectionCCW = [1; 0; 1; 0] ; % bool to reverse motor spin direction around 'thrustAxes'.
[p.propulsion] = definePropulsionModel(qRP); 

% aero data
p.aero.area = 1 ; % square meters
p.aero.mac = 0.25 ; % [m]
load('aeroLUT_NACA0012','aeroLUT_NACA0012') ; 
p.aero.LUT = aeroLUT_NACA0012 ; % name of .mat file that contains the aerodata

% Enviromental params
p.environ.g = -9.81 ; % World Coords is XYZ = [East, North, Up], i.e. gravity is a negative number
p.environ.rho = 1.225 ; % air density during flight (kg/m^3) 