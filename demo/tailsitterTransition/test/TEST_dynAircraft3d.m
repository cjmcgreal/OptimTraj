[p] = definePlantModel();

%% no thrust
n_time = 10 ; 
n_motors = numel(p.propulsion); 

z = zeros(12,n_time) ; 
u = zeros(n_motors,n_time) ; 

dz = dynAircraft3d(z, u, p) ; 


%% motors at 100% throttle
n_time = 10 ; 
n_motors = numel(p.propulsion); 

z = zeros(12,n_time) ; 
u = ones(n_motors,n_time) ; 

dz = dynAircraft3d(z, u, p)  


%% motors at 100% throttle; vertical climb at 20 m/s
n_time = 10 ; 
n_motors = numel(p.propulsion); 

z = zeros(12,n_time) ; 
z(9,:) = 20*ones(1,n_time) ; 
u = ones(n_motors,n_time) ; 

dz = dynAircraft3d(z, u, p) 

%% motors at 100% throttle; vertical climb
n_time = 10 ; 
n_motors = numel(p.propulsion); 

z = zeros(12,n_time) ; 
% (pi/2)-deg2rad(5)
z(9,:) = 10*ones(1,n_time) ; 
u = ones(n_motors,n_time) ; 

dz = dynAircraft3d(z, u, p) 