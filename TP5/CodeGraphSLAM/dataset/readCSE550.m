function scanList = readCSE550(number )
% Reading and formating simulated CSE550 dataset
%   Detailed explanation goes here

switch nargin
    case 0
        number=1566;
end


disp('Reading CSE550 dataset');

file=fopen('CSE550/scanOdom.txt');

scanList=[];
previousOdom = [1 0 2.4831];
previousPose = [1 0 2.4831];

while ~feof(file)
    % Reading raw data
    textscan(file,'ranges: [');
    laserData=cell2mat(textscan(file,'%f,'))';
    textscan(file,']');
    odomData=cell2mat(textscan(file,'%f'))';
    
    % Replace max range readings
    laserData(laserData==5.0)=Inf;
    
    % Converting quaternion to yaw angle
    siny_cosp = 2.0 * (odomData(7) * odomData(6) + odomData(4) * odomData(5));
    cosy_cosp = 1.0 - 2.0 * (odomData(5) * odomData(5) + odomData(6) * odomData(6));
    yaw = atan2(siny_cosp, cosy_cosp);
    
    % Adding noise to odometry
    d=norm([odomData(1)- previousOdom(1) odomData(2)- previousOdom(2)]);
    theta = atan2(odomData(2)- previousOdom(2),odomData(1)- previousOdom(1))-previousOdom(3);
    dyaw=yaw-previousOdom(3);
    
    newPose(1) = previousPose(1) + d*cos(previousPose(3)+theta);
    newPose(2) = previousPose(2) + d*sin(previousPose(3)+theta);
    newPose(3) = previousPose(3) + dyaw+0.0015*rand()+0.002*randn();
    
    % Interpolate to improve resolution
    interpData=zeros(1,4*size(laserData,2));
    interpData(1:4:end)=laserData;
    difference = abs(laserData(1:(end-1))-laserData(2:end))>0.3;
    interValues=[(3*laserData(1:(end-1))+laserData(2:end))/4 laserData(end)];
    interValues(difference)=Inf;
    interpData(2:4:end) = interValues;
    interValues=[(laserData(1:(end-1))+laserData(2:end))/2 laserData(end)];
    interValues(difference)=Inf;
    interpData(3:4:end) = interValues;
    interpData(4:4:end)=[(laserData(1:(end-1))+3*laserData(2:end))/4 laserData(end)];
    interValues(difference)=Inf;
    interpData(2:4:end) = interValues;
    
    % Create scan list
    scanList=[scanList createScan(interpData,[-3*pi/4:6*pi/(4*359):3*pi/4],newPose)];
    textscan(file,'magnum %f');
    
    previousOdom = [odomData([1 2]) yaw];
    previousPose = newPose;
    
    if size(scanList,2)>=number
        break
    end
    
end
fclose(file);

disp(['Finished reading ' num2str(size(scanList,2)) ' scans']);

end

