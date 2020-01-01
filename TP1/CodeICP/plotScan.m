function  plotScan(scan,color)
%displays a scan object
%   Detailed explanation goes here

switch nargin
        case 2
            c = color;
        case 1
            c = rand(1,3);
end
valid=(scan.ranges < 5);
scatter(scan.x(valid),scan.y(valid),2,c);
hold on
plot([scan.pose(1) scan.pose(1)+0.1*cos(scan.pose(3))],[scan.pose(2) scan.pose(2)+0.1*sin(scan.pose(3))],'color',c,'linewidth',3);
end

