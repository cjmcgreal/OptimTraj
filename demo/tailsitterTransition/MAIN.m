
% define aeromodel data
aeroModelName = 'exampleAeroModel' ; 
mac = 0.25 ; % [m]

% link data
p.areas = 1 ; % square meters
p.cps = zeros(1,3) ; % [m]
p.LUT = [[-180:1:180]' ; 

% Function Call
[aeroModel] = defineAeroModel(p,mac,aeroModelName); 