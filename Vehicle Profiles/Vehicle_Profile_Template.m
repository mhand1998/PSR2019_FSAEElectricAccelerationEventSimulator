clc
close
clear   %crtical to only get relevant variables in vehicle profile

%Fill in the variables below

m = ;                    %mass [kg]
g = ;                    %gravity [m/s^2]
wdia = ;             %wheel Diamter[m]
wb = ;                   %wheel Base[m]
CGz = ;                   %height of Center of Gravity[m]
swd = ;                  %weight distribution (rearward bias) [] 
ld = ;                   %lift distripbution (reward bias) [] 
mu = ;                  %longitudinal [] 
cd = ;                 %drag coefficent []
cl = ;                 %lift coefficent []
ref_area = ;           %refference area [m^2] 
air_density = ;        %air density[kg/m^3]
rolling_resistance = ; %rolling resistance[] 
powertrain_effeiency = ; %powertrain efficency[]
inverter_efficency = ;  %interter efficency[]