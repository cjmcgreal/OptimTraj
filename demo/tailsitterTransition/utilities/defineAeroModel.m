function [aeroModel] = defineAeroModel(p,mac,aeroModelName) 
% [aeroModel] = make_LDP_PlantModel(areas, cps, ControlSurface_bools, PropWash_bools, ControlSurface_ids, PropWash_ids) 
%
% Creates a struct m file to be used with the lift drag plugin
%   (First tier fields of the resultant struct are inputs into i.e. "LDP_OnUpdate") 
% 
% Inputs:
%   aeroModelName = [string] name of struct.
%   mac = [scalar] mean aerodynamic chord [m]
%   p = [struct] aero model parameters 
%       .linkNames = [string] name of each link ('i.e. 'main wing') 
%       .areas = [nx1] area of each link element [m^2]
%       .cps = [nx3] XYZ location of center of pressure of each element. [m]
%       .isPropWash = [nx1] indicate whether or not link is in [Bool] 
%           propwash.
%       .PropWash_id = [nx1] id of propwash. This corresponds to an element id in the .propulsion struct.
%       .isControlSurface = [nx1] indicate whether or not link is [Bool]
%           control surface (i.e. subject to deflection).
%       .ControlSurface_id = [nx1] id of control surface input. Indexed from 1.
%
% Output:
%   aeroModel(i) = [struct] with following fields for each link
%       .aeroModelName = [string] name of aeromodel (i.e. 'Swift021_aero')
%       .links(i) = For each aerodynamic element, i, the following properties.
%           .linkName 		= [string] name of each link (i.e. 'main wing')
%           .area 		= [scalar][m^2] area of wing link.
%           .cp			= [1 x 3] [m] position, in gazebo body coordinates, of center of pressure.
%           .isPropWash 	= [bool] set to "1" if element is under propwash.  In that case, '.propWash_id must be defined.
%           .propWash_id	= [scalar] if isPropWash == 1, defines which 'propWash' to use.  This corresponds to an element id in the .propulsion struct.
%           .isControlSurface  	= [bool] set to "1" if element is a control surface. In that case 'controlSurface_id' must be defined.
%           .controlSurface_id 	= [scalar] if element is under propwash, defines which 'controlSurfaceDeflection' to use.
%       
% Written by Conrad McGreal 2020-02-19

n_links = numel(p.areas) ; 

for i=1:n_links % for all links
    if exist('linkNames','var') % give names if defined, otherwise assign default names
        aeroModel.links(i).linkName = linkNames(i) ; 
    else
        aeroModel.links(i).linkNames = strcat('link_',num2str(i)) ; 
    end
    aeroModel.links(i).area = p.areas(i) ;
    aeroModel.links(i).cp = p.cps(i,:) ;
    aeroModel.links(i).isPropWash = p.isPropWash(i) ;
    aeroModel.links(i).PropWash_id = p.PropWash_id(i) ;
    aeroModel.links(i).isControlSurface = p.isControlSurface(i) ; 
    aeroModel.links(i).ControlSurface_id = p.ControlSurface_id(i) ;
end

aeroModel.aeroModelName = aeroModelName ; 
aeroModel.mac = mac ; 
