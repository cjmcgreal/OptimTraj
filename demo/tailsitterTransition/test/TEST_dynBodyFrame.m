disp('Running: TEST_dynBodyFrame.m')

% define plant model to operate on
[p] = definePlantModel();
n_motors = numel(p.propulsion); 

%% Test 0 - No input force => no acceleration
disp('Test 0 - no input force')
n_time = 1; 
u = zeros(n_motors,n_time) ; 
z_body = zeros(9,1) ; 
[ds] = dynBodyFrame(z_body, u, p) ;

%% Test 1 - No input force => no acceleration
disp('Test 1 - zero throttle ''wide vector'' ') 
n_time = 10 ; 
u = zeros(n_motors,n_time) ; 
z_body = zeros(9,n_time) ; 
[ds] = dynBodyFrame(z_body,u, p) ;

%% Test 2 - All motors at max throttle => acceleration in positive Z direction [i.e. ds(3)]
disp('Test 2 - max throttle') 
n_time = 10 ; 
u = ones(n_motors,n_time) ; 
z_body = zeros(9,n_time) ; 
[ds] = dynBodyFrame(z_body,u, p) ; 

%% Test 3 - "wide" input vector.
disp('Test 3 - ramp throttle ''wide'' ') 
[p] = definePlantModel();
n_motors = numel(p.propulsion); 
n_time = 100 ;
u = repmat(linspace(0,1,n_time),n_motors,1); 
z_body = zeros(9,n_time) ; 
[ds] = dynBodyFrame(z_body,u, p) ;
% 
% %% Test 4 - only motor 1 (port side motor {at the moment}) 
% % should result in (pitch accel = 0, roll accel < 0, yaw accel < 0)
% disp('Test 4 - port motor only ; ramp') 
% u = zeros(n_motors,100) ;
% u(1,:) = linspace(0,1,100) ; 
% omega = zeros(3,100) ; 
% [ds] = dynBodyFrame(omega,u, p) ;
% 
% %% Test 5 - only motor 2 (nose motor {at the moment}) 
% % should result in (pitch accel > 0, roll accel = 0, yaw accel > 0)
% disp('Test 5 - nose motor only ; ramp') 
% u = zeros(n_motors,100) ;
% u(2,:) = linspace(0,1,100) ; 
% omega = zeros(3,100) ; 
% [ds] = dynBodyFrame(omega,u, p) ;

%% 
disp('TEST_dynBodyFrame concluded without errors')