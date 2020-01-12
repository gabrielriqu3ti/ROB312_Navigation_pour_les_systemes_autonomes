function [sorteddist,sortedId] = findClosestScan(map,s2)
% Return map scan ids sorted according to distance to s2 scan
%   Detailed explanation goes here

distances=zeros(1,size(map,2));
for a=1:size(map,2)
    distances(a) = norm(map(a).pose(1:2)-s2.pose(1:2))+abs(AngleWrap(map(a).pose(3)-s2.pose(3)))/15;
    % The following is to prevent matching scans with too large rotation
    % differences
    if abs(AngleWrap(map(a).pose(3)-s2.pose(3))) > pi/3
        distances(a)=distances(a)*2;
    end
end
[sorteddist,sortedId] = sort(distances);

end

