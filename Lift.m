function [LIFT_FORCE] = Lift(cl,reference_area,air_density,velocity)
%Lift: Uses the modern lift equation to calculate lift from airfoil.
%Orignally used for vehicle downforce calculations.
LIFT_FORCE = reference_area.*cl.*(air_density*velocity^2/2);

end
