function [plantModel] = definePlantModelSwift021SN018NoAero(plotflag)

%% Define environmental and plant model params
% Enviromental params
p.g = -9.81 ; % World Coords is XYZ = [East, North, Up], i.e. gravity is a negative number
p.rho = 1.225 ; % air density during flight (kg/m^3) 

% Inertial params
p.m = 12 ; % 19.6
p.I = [11.19 0 0; 0 1 0; 0 0 10.415] ; % inertia tensor coords: [belly port nose] 
p.cg = [-0.009 0 -0.0137] ; % (m) location of center of gravity SN018 12/23/2019 CAD Coords: [0.52, 0.23, 0.37] inches

%% Propulsion system
% Prop order is: [port, starboard, nose, tail] 
% propeller diameters (m)
qRP.d_prop = [      
    0.3810
    0.3810
    0.6604
    0.6604
]; 

% thrust at 100% throttle (N)
qRP.maxThrust = [   
    18
    18
    78.1
    69.4
    ]; 

% RPM at 100% throttle (RPM) (currently from the ether)
qRP.maxRPM = 10000*ones(4,1) ; 

% torque at 100% throttle (Nm) % Currently made up parameters
qRP.maxTorque = [ones(2,1); 3*ones(2,1)] ;  

% motor locations (each row one motor in coords: [belly, port, nose] 
qRP.thrustLocations = [ 
    0.0204  0.5334  -0.1103 % port
    0.0204 -0.5334  -0.1103 % starboard
    0.0651  0        0.4470 % nose
   -0.1255  0       -0.4987 % tail
    ]; 

% thrust axes of each motor in coords: [starboard, belly, nose]  
qRP.thrustAxes = [
    0 0 1                   % port
    0 0 1                   % starboard
    -sind(7) 0 cosd(7)      % nose
    -sind(4.4) 0 cosd(4.4)  % tail
   ];
    
% bool to reverse motor spin direction around 'thrustAxes'.
qRP.isSpinDirectionCCW = [1; 0; 0; 1] ; % port, starboard, nose, tail

% Call function that creates the propulsion plant model
[p.propulsion] = definePropulsionModel(qRP) ; 

plantModel = p ; 

% Plot newly created model
if ~exist('plotflag','var')
    plotflag = 0 ; 
end 
if plotflag
    showPropulsionModel(p.propulsion)
end
