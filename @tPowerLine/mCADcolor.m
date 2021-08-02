function mCADcolor(drone,color)
% Modify tower base color

if nargin > 1
    drone.pCAD.i3D{length(drone.pCAD.obj)+1}.MarkerFaceColor = color';
end

end