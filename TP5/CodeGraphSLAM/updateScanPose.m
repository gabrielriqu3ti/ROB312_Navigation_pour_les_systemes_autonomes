function  scan = updateScanPose(scan,pose)
%transform a scan object
%   Detailed explanation goes here

scan.pose=pose;

scan.x = scan.pose(1) + scan.ranges.*cos(scan.angles+scan.pose(3));
scan.y = scan.pose(2) + scan.ranges.*sin(scan.angles+scan.pose(3));

end

