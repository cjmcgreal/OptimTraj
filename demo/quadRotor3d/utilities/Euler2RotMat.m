function [R] = Euler2RotMat(eul)
% [R] = Euler2RotMat(theta_x, theta_y, theta_z)
%
% Stand-in for eul2rotm (built-in function MATLAB Robotics System Toolbox)
% Converts Euler angles [rad] to 3x3 Rotation Matrix
%
% This implementation based on:
% S.M.LaValle, Planning Algorithms, Cambridge University Press, 2006, p.98 
% (PDF available here: http://planning.cs.uiuc.edu/ch3.pdf)
% as cited in: http://nghiaho.com/?page_id=846 
%
% Inputs:
%   eul = [n x 3] - Euler Rotation Angles
%
% Outputs:
%   R = [3 x 3 x n] - rotation matrix. 
%
% Written by Conrad McGreal @ 2019
R = zeros(3,3,size(eul,1)) ; 

for i=1:size(eul,1) 
    theta_x = eul(i,1) ; 
    cx = cos(theta_x) ; 
    sx = sin(theta_x) ; 
    
    theta_y = eul(i,2) ; 
    cy = cos(theta_y) ; 
    sy = sin(theta_y) ; 
    
    theta_z = eul(i,3) ;
    cz = cos(theta_z) ; 
    sz = sin(theta_z) ; 

    X = [1 0 0
        0 cx -sx
        0 sx cx];

    Y = [cy 0 sy
        0 1 0
        -sy 0 cy];

    Z = [cz -sz 0
        sz cz 0
        0 0 1];

    R(:,:,i) = Z*Y*X;
end