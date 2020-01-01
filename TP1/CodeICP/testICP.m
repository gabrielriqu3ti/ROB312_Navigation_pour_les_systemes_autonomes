% Test ICP localisation
%  Apply a random displacement to a scan and check the error of the recovered position through ICP


% Reading some data
cd 'dataset'
scanList=readU2IS(56);
cd '..'
scanOriginal = scanList(56);
scanTruePose = [0.3620 0.0143 0.0483]; % Manual estimation for scan 56 of U2IS dataset
refscan = scanList(1);

% Initialise error log
error =[];

for a=1:25
    
    % Generate random displacement and applies it to the second scan
    randT = 0.3*rand(2,1);
    randR = 0.2*rand(1,1)-0.05;
    scan= transformScan(scanOriginal,[cos(randR) -sin(randR);sin(randR) cos(randR)],randT);
    
    
    % Displays initial positions
    close all
    plotScan(refscan,'b');
    hold on
    plotScan(scan,'r');
    
    
    % perform ICP
    [R,t]=icp(refscan,scan,200,1e-7);
    
    % Appli motion to scan
    
    scan= transformScan(scan,R,t);
    error = [error ; scan.pose - scanTruePose];
    
    % Display
    plotScan(scan,'g');
    legend('Ref Scan','Ref Scan origin','Scan before ICP','Scan before ICP origin','Scan after ICP','Scan after ICP origin');
end

disp(['Translation error :' num2str(mean(sqrt(error(:,1).^2+error(:,2).^2)))]);
disp(['Rotation error :' num2str(mean(abs(error(3)),'all'))]);