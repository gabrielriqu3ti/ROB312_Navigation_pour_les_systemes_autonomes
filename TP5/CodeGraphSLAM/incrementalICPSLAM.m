% Incremental ICP SLAM 
%  Computes position of each scan with respect to closest one in the
%  current map and add scan to the map if it is far enough of all the
%  existing ones


%% Reading data
cd 'dataset'
% scanList=readFR079();
% scanList=readCSE550();
scanList=readU2IS();
cd '..'

%% Parameters for scan processing
minScan=280; % 1;
step=3;
maxScan= 640; % size(scanList,2)-step;

%% Parameters for map building
distThresholdAdd=0.25;

%% Copy for reference display and map init
odomScanList=scanList;
map=scanList(minScan);

%% Init displays
close all
subplot(1,2,1)
hold off
plotScan(odomScanList(minScan));
hold on
subplot(1,2,2)
hold off
plotScan(map);
hold on


for a=(minScan+step):step:maxScan
    disp(['Processing scan ' num2str(a)]);
    s2=scanList(a);
    
    [sorteddist,sortedId]=findClosestScan(map,s2);
    refScanId = sortedId(1);
    disp(['Reference scan : ' num2str(refScanId)]);
    
    %% perform ICP with closest scan
    [R,t]=icp(map(refScanId),s2,200,1e-7);
    
    %% Correct all future scans odometry pose
    for b=a:step:maxScan
        scanList(b)= transformScan(scanList(b),R,t);
    end
    
    %% Add scan to map if it is far enough
    if sorteddist(1) > distThresholdAdd
        map= [map scanList(a)];
     end
    
    
    %% Display
    subplot(1,2,1)
    plotScan(odomScanList(a));
    axis equal
    title('Données brutes');
    
    subplot(1,2,2)
    plotScanList(map);
    axis equal
    title('incremental ICP SLAM map');
    
    drawnow
    disp(['Map size : ' num2str(size(map,2))]);
    %pause
    
end