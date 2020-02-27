
% Define propulsion model
qRP.d_prop = 0.305*ones(4,1) ; % propeller diameter (m)
qRP.maxThrust = 25*ones(4,1) ; % thrust at 100% throttle (N)
qRP.maxRPM = 10000*ones(4,1) ; % RPM at 100% throttle (RPM)
qRP.maxTorque = ones(4,1) ;  % torque at 100% throttle (Nm)
qRP.thrustLocations = [0.5 0 0; 0 0.5 0; -0.5 0 0; 0 -0.5 0]; % motor locations (each row one motor in coords: [port, nose, top] 
qRP.thrustAxes = repmat([0 0 1],4,1) ; % thrust axes of each motor in coords port, nose, top.
qRP.isSpinDirectionCCW = [1; 0; 1; 0] ; % bool to reverse motor spin direction around 'thrustAxes'.
[p.propulsion] = definePropulsionModel(qRP); 
p.environ.rho = 1.225 ; 
p.inertial.cg = zeros(1,3) ; 

% control vector
u = zeros(numel(p.propulsion),1) ; 

% Call function
% [forces, moments] = calculatePropulsionWrenches(u, p);

%% 'wide' input vector
% control vector
ramp = linspace(0,1,10); % ramp throttles
u = repmat(ramp,numel(p.propulsion),1); 

% Call function
[forces, moments] = calculatePropulsionWrenches(u, p); 