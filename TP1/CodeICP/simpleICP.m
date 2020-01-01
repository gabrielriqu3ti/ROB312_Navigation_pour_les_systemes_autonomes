% Simple ICP localisation demo
%  Compute position of each scan with respect to the previous one


% Reading data
cd 'dataset'
% scanList=readFR079();
% scanList=readCSE550();
 scanList=readBicocca();
% scanList=readU2IS();
cd '..'

% Copy for reference display
odomScanList=scanList;

% Parameters for scan processing
minScan=1;
step=3;
maxScan=size(scanList,2)-step;

% Init displays
close
subplot(1,2,1)
hold off
plotScan(odomScanList(minScan));
hold on
subplot(1,2,2)
hold off
plotScan(scanList(minScan),'r');
hold on



for a=minScan:step:maxScan
    s1=scanList(a);
    s2=scanList(a+step);
    
    % perform ICP
    [R,t]=icp(s1,s2,200,1e-7);
    
    % correct future scans 
    for b=(a+step):step:maxScan
        scanList(b)= transformScan(scanList(b),R,t);
    end    
    
   % Display
    subplot(1,2,1)
    plotScan(odomScanList(a+step));
    subplot(1,2,2)
    plotScan(transformScan(s2,R,t));
    drawnow
    
end