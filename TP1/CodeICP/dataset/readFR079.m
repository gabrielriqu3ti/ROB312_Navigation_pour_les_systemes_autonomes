function scanList = readFR079(number)
% Reading and formating FR079 dataset from carmen log format
%   Detailed explanation goes here

switch nargin
    case 0
        number=4919;
end

disp("Reading FR079 dataset");

file=fopen('fr079/laserData.txt');

textscan(file,"# FLASER num_readings [range_readings] x y theta odom_x odom_y odom_theta");

scanList=[];
while ~feof(file)
    textscan(file,"FLASER 360");
    rawData=cell2mat(textscan(file,"%f"))';
    
    scanList=[scanList createScan(rawData(1:360),[-pi/2:pi/359:pi/2],rawData(361:363))];
    textscan(file,"magnum %f");
    
    if size(scanList,2)>=number
        break
    end
    
end
fclose(file);

disp(['Finished reading ' num2str(size(scanList,2)) ' scans']);

end

