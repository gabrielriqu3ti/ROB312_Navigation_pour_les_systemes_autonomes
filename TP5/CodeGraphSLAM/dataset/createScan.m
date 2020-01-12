function [scan] = createScan(ranges,angles, pose)
%createScan creates a scan struct from its components
%   ranges : array of scan distances
%   angles : angles of each reading
%   pose : absolute pose as array [x y theta]

scan.ranges = ranges;
scan.angles = angles;
scan.pose = pose;
scan.x = pose(1) + ranges.*cos(angles+pose(3));
scan.y = pose(2) + ranges.*sin(angles+pose(3));

end

