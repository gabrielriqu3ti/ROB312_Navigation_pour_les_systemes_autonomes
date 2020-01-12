function angle = meanAngle(angleList)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

mcos=mean(cos(angleList));
msin=mean(sin(angleList));

angle=atan2(msin,mcos);

end

