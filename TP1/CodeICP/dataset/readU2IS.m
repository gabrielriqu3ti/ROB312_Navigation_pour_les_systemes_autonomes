function scanList = readU2IS(number)
% Reading and formating U2IS dataset
%   Detailed explanation goes here

switch nargin
        case 0
            number=855;
end

disp("Reading U2IS dataset");

fileLaser=fopen('U2IS/laser_filt.txt');
fileOdom=fopen('U2IS/odom_filt.txt');

scanList=[];
odomTime=cell2mat(textscan(fileOdom,"secs:%d"))';
odomTime=[odomTime cell2mat(textscan(fileOdom,"nsecs:%d"))'];
odomData=cell2mat(textscan(fileOdom,"%f",13))';

angles=[-2.35619449615:0.00436332309619:2.35619449615];
%odomTimes=[];
%laserTimes = [];
while ~feof(fileOdom)
    % Reading raw data
    laserTime=cell2mat(textscan(fileLaser,"secs:%d"))';
    %laserTimes=[laserTimes mod(laserTime,1000)];
    laserTime=[laserTime cell2mat(textscan(fileLaser,"nsecs:%d"))'];
    textscan(fileLaser,"ranges: [");
    laserData=cell2mat(textscan(fileLaser,"%f,"))';
    textscan(fileLaser,"]");
    
    while odomTime(1) > laserTime(1)
        laserTime=cell2mat(textscan(fileLaser,"secs:%d"))';
        laserTime=[laserTime cell2mat(textscan(fileLaser,"nsecs:%d"))'];
        textscan(fileLaser,"ranges: [");
        laserData=cell2mat(textscan(fileLaser,"%f,"))';
        textscan(fileLaser,"]");
    end
    
    if odomTime(1) == laserTime(1)
        while odomTime(2) > laserTime(2)
            laserTime=cell2mat(textscan(fileLaser,"secs:%d"))';
            laserTime=[laserTime cell2mat(textscan(fileLaser,"nsecs:%d"))'];
            textscan(fileLaser,"ranges: [");
            laserData=cell2mat(textscan(fileLaser,"%f,"))';
            textscan(fileLaser,"]");
        end
    end
    
    
    % Replace max range readings and remove borders
    laserData(laserData>20.0)=Inf;
    laserData([1:80])=Inf;
    laserData([end-80:end])=Inf;
    
    % Adding some noise to laser readings
    %laserData=laserData+0.005*randn(1,90);
    
    % Converting quaternion to yaw angle
    siny_cosp = 2.0 * (odomData(7) * odomData(6) + odomData(4) * odomData(5));
    cosy_cosp = 1.0 - 2.0 * (odomData(5) * odomData(5) + odomData(6) * odomData(6));
    yaw = atan2(siny_cosp, cosy_cosp);
    
    % Compute position of laser
    
    pose = [odomData(1)+0.1*cos(yaw) odomData(2)+0.1*sin(yaw) yaw];
    % Adding noise to odometry
    % TODO
    
    % Create scanlist
    scanList=[scanList createScan(laserData(1:2:end),angles(1:2:end),pose)];
%    scanList=[scanList createScan(laserData(1:2:end),angles(1:2:end),[0 0 0])];
    
    % reading next odom
    odomTime=cell2mat(textscan(fileOdom,"secs:%d"))';
    %odomTimes=[odomTimes mod(odomTime,1000)];
    odomTime=[odomTime cell2mat(textscan(fileOdom,"nsecs:%d"))'];
    odomData=cell2mat(textscan(fileOdom,"%f",13))';
    
    if size(scanList,2)>=number
        break
    end

end
fclose(fileLaser);
fclose(fileOdom);
disp(['Finished reading ' num2str(size(scanList,2)) ' scans']);

% hold off
% plot(odomTimes);
% hold on
% plot(laserTimes);
% pause

end

