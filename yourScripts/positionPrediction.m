function [ poseOut, covOut ] = positionPrediction( poseIn,covIn,delSr,delSl)
%[poseOut, covOut] = POSITIONPREDICTION(poseIn,covIn,delSr,delSl) perform
%one step of robot pose prediction from a set of wheel displacements
%   poseIn = old robot pose
%   covIn = uncertainty on the old robot pose
%   delSr = right wheel linear displacement
%   delSl = left wheel linear displacement


%% Constants
% The robot parameters are read globally, odoB is the wheel separation, kR
% and kL are the odometry uncertainty parameters
global odoB kR kL 
global odoB_kf kR_kf kL_kf

%% pose update

% poseOut = [0;0;0];
A = eye(3,3);
sum_dl_dr = delSr + delSl;
sum_dl_dr_hal = sum_dl_dr * 0.5;
sub_dr_dl = delSr - delSl;
angular_dis = sub_dr_dl * 0.5 / odoB_kf;
poseOut = A * poseIn + [sum_dl_dr_hal * cos(poseIn(3)+angular_dis);
                    sum_dl_dr_hal * sin(poseIn(3)+angular_dis);
                    angular_dis * 2];

poseOut(3) = atan2(sin(poseOut(3)),cos(poseOut(3)));
% if (poseOut(3) > pi)
%     poseOut(3) = poseOut(3) - 2*pi;
% elseif poseOut(3)<-pi
%     poseOut(3) = poseOut(3) + 2*pi;
% end

%% Covariance update
Fx = eye(3,3) + [0 0 -sum_dl_dr_hal*sin(poseIn(3)+angular_dis);
                 0 0  sum_dl_dr_hal*cos(poseIn(3)+angular_dis);
                 0 0 0];
Fu = [0.5*cos(poseIn(3)+angular_dis)-sum_dl_dr_hal*sin(poseIn(3)+angular_dis)/2/odoB_kf ...
      0.5*cos(poseIn(3)+angular_dis)+sum_dl_dr_hal*sin(poseIn(3)+angular_dis)/2/odoB_kf;
      0.5*sin(poseIn(3)+angular_dis)+sum_dl_dr_hal*cos(poseIn(3)+angular_dis)/2/odoB_kf ...
      0.5*sin(poseIn(3)+angular_dis)-sum_dl_dr_hal*cos(poseIn(3)+angular_dis)/2/odoB_kf;
      1/odoB_kf -1/odoB_kf];

covOut = Fx*covIn*Fx'+Fu*[kR_kf*abs(delSr) 0;0 kL_kf*abs(delSl)]*Fu';

% covOut = zeros(3,3);

end
