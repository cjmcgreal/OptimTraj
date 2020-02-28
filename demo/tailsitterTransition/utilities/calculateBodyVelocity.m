function [bodyVelocity] = calculateBodyVelocity(worldOri,worldLinearVelocity)
% [bodyVelocity] = calculateBodyVelocity(worldOri,worldLinearVelocity)
%
% Converts from 'world frame' to 'body frame'.
%
% Inputs: 
%   worldOri = [3x1][rad] world orientation typically [roll, pitch, yaw]
%   worldLinearVelocity = [3x1][m/s] world linear velocity. [often East, North, Up]
%
% Outputs:
%   bodyVelocity = [3x1][m/s] velocity in bodyframe.
%
% Created by Conrad McGreal 2019-06-04

BodyRotMat = Euler2RotMat(worldOri(1), worldOri(2), worldOri(3)) ;  %
bodyVelocity = (worldLinearVelocity * BodyRotMat) ;