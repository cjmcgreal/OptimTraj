
Simple aero model accounts for lift and drag forces in the 'forward velocity' direction only (i.e. chordwise flow). 

In other words, no effects are considered due to spanwise flow. 

With respect to the chordwise flow, the following is defined.

Aerodynamic Coefficient Lookup table (.LUT) is size (360,4) i.e it consists of 4 column vectors: [alpha CL CD CM]

alpha = angle-of-attack = angle between chord line and forward velocity (i.e. oncoming air) of aircraft.
Angle of attack is considered zero when oncoming airflow is parallel with chord line.
Pitch "nose up" is positive angle of attack.

CL = lift coefficient ; see: https://www.grc.nasa.gov/WWW/K-12/airplane/liftco.html
CD = drag coefficient
CM = moment coefficient
	dCm/dalpha assumed constant here -0.005 