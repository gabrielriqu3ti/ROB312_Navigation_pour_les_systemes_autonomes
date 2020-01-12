% Graph ICP SLAM demo
%  Computes position of each scan with respect to several scans in the
%  current map


% Reading data
cd 'dataset'
% scanList=readFR079();
% scanList=readCSE550();
scanList=readU2IS();
cd '..'

% Parameters for scan processing
minScan=280; % 1;
step=3;
maxScan= 640; % size(scanList,2)-step;

% Parameters for map building
distThresholdMatch=0.6;
distThresholdAdd=0.25;

% Copy for reference display and map init
odomScanList=scanList;
map=scanList(minScan);

%initialize graph of relative positions
Graphtx=[0];
Graphty=[0];
Graphtheta=[0];

% Init displays
close all
subplot(1,2,1)
hold off
plotScan(odomScanList(minScan));
hold on
subplot(1,2,2)
hold off
plotScan(map);
hold on

% Process scans
for a=(minScan+step):step:maxScan
    
    disp(['Processing scan ' num2str(a)]);
    s2=scanList(a);
    
    % get list of map scan sorted by distance
    [sorteddist,sortedId]=findClosestScan(map,s2);
    
    % Keep only the ones below the distance threshold, or the closest one
    closeScans = sortedId (sorteddist<distThresholdMatch);
    disp(['Reference scans : ' num2str(closeScans)]);
    if isempty(closeScans)
        closeScans = sortedId(1);
    end
    
    % perform ICP with closest scan just to correct future odometry and add
    % to map
    [R,t]=icp(map(closeScans(1)),s2,200,1e-7);
    
    % Correct all future scans
    for b=a:step:maxScan
        scanList(b)= transformScan(scanList(b),R,t);
    end
    
    % --- Add scan to map and update graph if needed
    if sorteddist(1) > distThresholdAdd
        % add scan at the end of the map list
        map= [map scanList(a)];
        id2=size(map,2);
        s2=map(id2);
        
        % Build graph
        for i=1:size(closeScans,2)
            
            % take the reference scan among the closest map scan
            idi=closeScans(i);
            s1=map(idi);
            
            % compute position wrt the ref scan
            [Ri,ti]=icp(s1,s2,200,1e-7);
            
            % compute position of new scan wrt reference scan
            si=transformScan(s2,Ri,ti); % absolute pose
            
            deltatheta=AngleWrap(si.pose(3)-s1.pose(3)); % relative pose
            deltat=si.pose(1:2)'-s1.pose(1:2)';
            
            % Add relative position in the graph
            Graphtx(idi,id2)=deltat(1);
            Graphty(idi,id2)=deltat(2);
            Graphtheta(idi,id2)=deltatheta;
            
            % make graph symetric
            Graphtx(id2,idi)=-Graphtx(idi,id2);
            Graphty(id2,idi)=-Graphty(idi,id2);
            Graphtheta(id2,idi)=-Graphtheta(idi,id2);
            
        end
        
        % --- Optimize graph until updates fall below threshold
        updateMax=1;
        updateNB=0;
        while updateMax > 1e-4
            updateMax=0;
            updateNB = updateNB +1;
            for i=1:size(map,2)
                % Create a list of scan pose computed through neighbor pose and
                % relative position
                newPoseList=[];
                for j=1:size(map,2)
                    if Graphtx(j,i)
                        newAngle = AngleWrap(map(j).pose(3) + Graphtheta(j,i));
                        newX = map(j).pose(1) + Graphtx(j,i);
                        newY = map(j).pose(2) + Graphty(j,i);
                        newPoseList = [newPoseList [newX;newY;newAngle]];
                    end
                end
                
                % Compute pose as mean of positions from neighbors
                newPose(1)= mean(newPoseList(1,:));
                newPose(2)= mean(newPoseList(2,:));
                newPose(3)= meanAngle(newPoseList(3,:));
                updateMax = max([abs(newPose - map(i).pose) updateMax]);
                map(i)=updateScanPose(map(i),newPose);                
            end
        end

       disp(['Map size : ' num2str(size(map,2)) ', Graph Updates : ' num2str(updateNB)]);
    end
    
    
    % Display
    subplot(1,2,1)
    plotScan(odomScanList(a));
    axis equal
    title('Données brutes');
    
    subplot(1,2,2)
    plotScanList(map);
    axis equal
    title('graph ICP SLAM map');
    
    drawnow
    %pause
    
end