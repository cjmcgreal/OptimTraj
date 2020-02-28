[p] = definePlantModel();

n_time = 10 ; 
n_motors = numel(p.propulsion); 

z = zeros(12,n_time) ; 
u = zeros(n_motors,n_time) ; 

dz = dynAircraft3d(z, u, p) ; 