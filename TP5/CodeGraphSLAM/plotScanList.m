function  plotScanList(scanList)
%displays an array of scan object
%   Detailed explanation goes here
    
hold off
plotScan(scanList(1));
hold on
for i=2:size(scanList,2)
    plotScan(scanList(i));

end

end

