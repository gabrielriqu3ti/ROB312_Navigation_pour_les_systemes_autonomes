function [zf,idf, zn]= data_associate(x,P,z,R, gate1, gate2)
% 
% Simple gated nearest-neighbour data-association. No clever feature
% caching tricks to speed up association, so computation is O(N), where
% N is the number of features in the state.
%
% Tim Bailey 2004.
% zf : observation of already known feature
% idf : id of already known feature
% zn : observation of new feature
%
% x : state vector
% P : covariance matrix of state vector
% z : observations
% R : observation noise covariance
% gate1 : threshold for correct association
% gate2 : threshold for new feature


