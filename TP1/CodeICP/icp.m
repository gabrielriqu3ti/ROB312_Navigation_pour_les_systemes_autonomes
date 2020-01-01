function [R,t] = icp(model,data,maxIter,thres)
% ICP (iterative closest point) algorithm
%  Simple ICP implementation for teaching purpose
%  model : scan taken as the reference position
%  data : scan to align on the model
%  maxIter : maximum number of ICP iterations
%  thres : threshold to stop ICP when correction is smaller

%disp('Running ICP');

% Various inits
olddist = 1e9;
maxRange=10; % limit on the distance of points used for ICP

% Create array of x and y coordinates of valid readings for reference scan
valid = model.ranges<maxRange;
ref = [model.x;model.y];
ref = ref(:,valid);

% Create array of x and y coordinates of valid readings for processed scan
valid = data.ranges<maxRange;
dat = [data.x;data.y];
dat = dat(:,valid);

% Filter data points 

% TODO : filter point contained in 'dat' variable

% Initialize transformation to identity
R=eye(2);
t=zeros(2,1);

% Main ICP loop
for iter=1:maxIter
        
    % ----- Compute point association    
    
    % TODO : find point of 'ref' associated to each point of 'dat'
    % 'meandist' should be the mean distance to associated points
    
    meandist=0;
    
    % ----- Filter associations
    
    % TODO : remove unwanted point associations
    
    % ----- Compute transform
    
    % TODO : compute Ri & Ti that minimize errors given the associated
    % points
    
    Ri=eye(2);
    Ti=zeros(2,1);    
    
    % ----- Apply transformation to points for next iteration
    dat=Ri*dat;                       
    dat=dat+Ti;
    
    % ----- Update global transformation
    R=Ri*R;    
    t=Ri*t+Ti;
    
    % ----- Stop when no more progress
    if abs(olddist-meandist) < thres
        break
    end
    
    % store mean residual error to check progress
    olddist=meandist;
    
end

%disp(['Finished ICP with error :' num2str(meandist)]);



