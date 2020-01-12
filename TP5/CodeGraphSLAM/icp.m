function [R,t] = icp(model,data,maxIter,thres)
% ICP (iterative closest point) algorithm
%  Simple ICP implementation for teaching purpose
%  model : scan taken as the reference position
%  data : scan to align on the model
%  maxIter : maximum number of ICP iterations
%  thres : threshold to stop ICP when correction is smaller

%disp('Running ICP');

% Various inits
olddist=9e99; % residual error
maxRange=10; % limit on the distance of points used for ICP
minResolution = 0.01; % min spacing between 2 points used for ICP

% Create array of x and y coordinates of valid readings for reference scan
valid = model.ranges<maxRange;
ref = [model.x;model.y];
ref = ref(:,valid);

% Create array of x and y coordinates of valid readings for processed scan
valid = data.ranges<maxRange;
dat = [data.x;data.y];
dat = dat(:,valid);

% Filter data points too close to each other
prevPt=dat(:,1);
datv = dat(:,1);
for a=2:size(dat,2)
   if norm(prevPt-dat(:,a))> minResolution
       datv=[datv dat(:,a)];
       prevPt=dat(:,a);
   end
end
dat=datv;

% Initialize transformation to identity
R=eye(2);
t=zeros(2,1);

% Main ICP loop
for iter=1:maxIter
        
    % ----- Find nearest Neighbors for each point    
    for i=1:size(dat,2)
        minval=9e99;
        for j=1:size(ref,2)
            val=norm(dat(:,i)-ref(:,j));
            if val<minval
                minval=val;
                index(i)=j;  % store index of matching point
                dist(i)=val; % store distance of matching point
            end
        end
    end
    
    % remove points whose neighbor distance is far above mean
    meandist=mean(dist.^2);
    valid = dist < (min(sqrt(meandist)+1e-3,0.5)); % only matching below 0.1 or mean error
    vdat=dat(:,valid);
    vdist=dist(valid);
    vindex=index(valid);
    meandist=mean(vdist.^2);
    
    % ----- Compute transform
    
    % Compute point mean
    mdat=mean(vdat,2);
    mref=mean(ref(:,vindex),2);
    
    
    % Use SVD for transform computation
    C=(vdat-mdat)*(ref(:,vindex)-mref)';
    [U,~,V]=svd(C);
    Ri=V*U';
    Ti=mref-Ri*mdat;
    
    % Apply transformation to points
    dat=Ri*dat;                       
    dat=dat+repmat(Ti,1,size(dat,2));
    
    % Update global transformation
    R=Ri*R;    
    t=Ri*t+Ti;
    
    % Stop when no more progress
    if abs(olddist-meandist) < thres
        break
    end
    
    % store mean residual error to check progress
    olddist=meandist;
    
end

%disp(['Finished ICP with error :' num2str(meandist)]);



