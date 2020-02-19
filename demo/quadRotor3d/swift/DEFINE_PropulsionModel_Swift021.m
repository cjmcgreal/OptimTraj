% defines propulsion model for Swift 021

% Define parameters

% Order is: [port, starboard, nose, tail] 

% propeller diameters (m)
qRP.d_prop = [      
    0.3810
    0.3810
    0.6604
    0.6604
]; 

% thrust at 100% throttle (N)
qRP.maxThrust = [   
    40
    40
    78.1
    69.4
    ]; 

% RPM at 100% throttle (RPM) (currently from the ether)
qRP.maxRPM = 10000*ones(4,1) ; 

% torque at 100% throttle (Nm) % Currently made up parameters
qRP.maxTorque = [ones(2,1); 3*ones(2,1)] ;  

% motor locations (each row one motor in coords: [starboard, belly, nose] 
qRP.thrustLocations = [ 
   -0.5334 0.0204 -0.1103
    0.5334 0.0204 -0.1103
    0      0.0651  0.4470
    0     -0.1255 -0.4987
    ]; 

% thrust axes of each motor in coords: [starboard, belly, nose]  
qRP.thrustAxes = [
    0 0 1
    0 0 1
    0 -sind(7) cosd(7)
    0 -sind(4.4) cosd(4.4)
   ];
    
% bool to reverse motor spin direction around 'thrustAxes'.
qRP.isSpinDirectionCCW = [1; 0; 0; 1] ; 

% Call function that creates the propulsion plant model
[PropulsionModelSwift021] = definePropulsionModel(qRP) ; 

% Plot newly created model
showPropulsionModel(PropulsionModelSwift021)

%% save2disk
save2disk = 1 ;
if save2disk
    saveoutfilename = 'PropulsionModelSwift021_SS' ; 
    save(saveoutfilename, 'PropulsionModelSwift021');
    disp('PropulsionModelSwift021.mat saved to disk')
end
