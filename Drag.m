function [DRAG_FORCE] = Drag(cd,frontal_area,air_density,velocity)
%Drag: Uses the drag equation to calculate drag on object
DRAG_FORCE = frontal_area.*cd.*(air_density.*velocity^2/2);

end

