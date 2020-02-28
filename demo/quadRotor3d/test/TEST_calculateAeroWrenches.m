% define plant model to operate on
[p] = definePlantModel();

%% single time step - no speed
n_time = 1 ; 
z_body = zeros(9,n_time) ; 
[forces, moments] = calculateAeroWrenches(z_body, p.aero); 

%% multiple time steps - no speed
n_time = 10 ; 
z_body = zeros(9,n_time) ; 
[forces, moments] = calculateAeroWrenches(z_body, p.aero);

%% multiple time steps - ramp speed in world z direction
[p] = definePlantModel();
n_time = 10 ; 
z_body = zeros(9,n_time) ;clc
z_body(6,:) = linspace(0,10,n_time) ; 
[forces, moments] = calculateAeroWrenches(z_body, p.aero, 1.225);

%% multiple time steps - ramp speed north
% If using [East, North, Up] and [starboard,belly,nose] this corresponds to
% cruising east in 'level flight'
[p] = definePlantModel();
n_time = 10 ; 
z_body = zeros(9,n_time) ;
z_body(1,:) = (-pi/2)+deg2rad(7) ; % level flight, nose pointed North
z_body(5,:) = linspace(0,25,n_time) ; % cruising North 
[forces, moments] = calculateAeroWrenches(z_body, p.aero, 1.225);

%% multiple time steps - ramp speed west
% If using [East, North, Up] and [starboard,belly,nose] this corresponds to
% cruising North in 'level flight'
[p] = definePlantModel();
n_time = 10 ; 
z_body = zeros(9,n_time) ;
z_body(1,:) = 0 ; % -pi/2; % nose to horizon (north) 
z_body(2,:) = 0 ; % -pi/2;  % nose to west
z_body(3,:) = pi/2;  % belly to west
z_body(4,:) = linspace(0,-10,n_time) ; % cruising East
[forces, moments] = calculateAeroWrenches(z_body, p.aero, 1.225);