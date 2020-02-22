%% Define example parameters

% model data
aeroModelName = 'exampleAeroModel' ; 
mac = 0.25 ; % [m]

% link data
p.areas = rand(10,1) ; % square meters
p.cps = rand(10,3) ; % [m]
p.isControlSurface = round(rand(10,1)) ; % which ones are control surfaces
p.ControlSurface_id = round(3*rand(10,1)) ;  % what are the id of the relevant control signal
p.isPropWash = round(rand(10,1)) ; % which elements are in propwash?
p.PropWash_id = round(3*rand(10,1)) ; % propwash of which motor?

% Function Call
[aeroModel] = defineAeroModel(p,mac,aeroModelName); 