function  scan = transformScan(scan,R,t)
%transform a scan object
%   Detailed explanation goes here


newXY = R*[scan.pose(1);scan.pose(2)]+t;
newTheta = scan.pose(3) + atan2(R(2,1),R(1,1));

scan.pose=[newXY(1), newXY(2), newTheta];

scan.x = scan.pose(1) + scan.ranges.*cos(scan.angles+scan.pose(3));
scan.y = scan.pose(2) + scan.ranges.*sin(scan.angles+scan.pose(3));


end

