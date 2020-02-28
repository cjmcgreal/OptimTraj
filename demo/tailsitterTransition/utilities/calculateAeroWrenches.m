function [Fr,moments] = calculateAeroWrenches(z_body, aeroModel, rho)
%
% INPUTS: 
%   z_body = [9,n] abbreviated state vector consists of [att; linVel;omega] where
%       eul = [pitch; roll; yaw][rad]
%       linVel = [dx; dy; dz] [m/s] world linear velocity
%       omega = [dpitch; droll; dyaw][rad/s] world angular velocity.
n = size(z_body,2) ; 

% unpack inputs
eul = z_body(1:3,:);
linVel  = z_body(4:6,:); 

% Convert world velocity and body orientation into body velocity.
BodyRotMat = Euler2RotMat(eul') ;  % create body rotation matrix
for i=1:size(BodyRotMat,3)
    bodyLinVel = linVel' * BodyRotMat(:,:,i)  ;
end

%% Convert into aero represenation (V & alpha) for coefficient lookup
% calculate V_inf for chord
chordLine = repmat(aeroModel.chord,n,1) ; % replicate chord line for dot product 
V_inf = dot(chordLine,bodyLinVel,2) ;     % velocity for the purposes of aerodynamic lookup

% calculate alpha
alphas = atan2(bodyLinVel(:,2),bodyLinVel(:,3)); 

% interpolate aero coefficients
alphas_deg = rad2deg(alphas); % convert to deg for LUT
CL = interp1(aeroModel.LUT(:,1),aeroModel.LUT(:,2),alphas_deg) ;
CD = interp1(aeroModel.LUT(:,1),aeroModel.LUT(:,3),alphas_deg) ;
CM = interp1(aeroModel.LUT(:,1),aeroModel.LUT(:,4),alphas_deg) ;
 
% % calculate lift and drag based on forward speed and alpha
A = aeroModel.area; 
q = 0.5 * rho * V_inf.^2; 
lift = CL .* q * A; 
drag = CD .* q * A; 
moment = CM .* q * A * aeroModel.mac ; 
 
%% Convert from aero frame back into body frame
% Find acting direction of aerodynamic forces
% drag is along direction of travel (but acting in the opposing direction)
dragUnit = -bodyLinVel'./vecnorm(bodyLinVel') ;
dragUnit(isnan(dragUnit))=0; % replace NaNs with zeros

% lift is perpendicular to direction of travel; define using the cross
% product with the span.
span = aeroModel.span ; 
manySpans = repmat(span',1,n) ; 
liftUnit = cross(dragUnit,manySpans); 
liftUnit(3,:) = -liftUnit(3,:) ; % TODO: remove this pending testing

% multiply drag forces by unit vector
dragVec = drag .* dragUnit';
liftVec = lift .* liftUnit';
 
% Calculate resultant force vector
Fr = dragVec + liftVec ; 

% Pack moment vector 
moments = [moment zeros(size(moment)) zeros(size(moment))] ; % TODO: remove this pending testing and replace with a dot product between the moment and the span axis.
