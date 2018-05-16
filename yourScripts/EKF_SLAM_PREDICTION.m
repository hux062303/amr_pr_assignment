function [poseAug, poseCovAug] = EKF_SLAM_PREDICTION(delSr, delSl, poseAug, poseCovAug)
%UNTITLED13 此处显示有关此函数的摘要
%   此处显示详细说明

%% Constants
% The robot parameters are read globally, odoB is the wheel separation, kR
% and kL are the odometry uncertainty parameters
global odoB_kf kR_kf kL_kf

%% pose update
A = eye(3,3);
sum_dl_dr = delSr + delSl;
sum_dl_dr_hal = sum_dl_dr * 0.5;
sub_dr_dl = delSr - delSl;
angular_dis = sub_dr_dl * 0.5 / odoB_kf;
poseOut = A * poseAug(1:3,1) + [sum_dl_dr_hal * cos(poseAug(3)+angular_dis);
                    sum_dl_dr_hal * sin(poseAug(3)+angular_dis);
                    angular_dis * 2];
poseOut(3) = atan2(sin(poseOut(3)),cos(poseOut(3)));

%% Covariance update
Fx = eye(3,3) + [0 0 -sum_dl_dr_hal*sin(poseAug(3)+angular_dis);
                 0 0  sum_dl_dr_hal*cos(poseAug(3)+angular_dis);
                 0 0 0];

Fu = [0.5*cos(poseAug(3)+angular_dis)-sum_dl_dr_hal*sin(poseAug(3)+angular_dis)/2/odoB_kf ...
      0.5*cos(poseAug(3)+angular_dis)+sum_dl_dr_hal*sin(poseAug(3)+angular_dis)/2/odoB_kf;
      0.5*sin(poseAug(3)+angular_dis)+sum_dl_dr_hal*cos(poseAug(3)+angular_dis)/2/odoB_kf ...
      0.5*sin(poseAug(3)+angular_dis)-sum_dl_dr_hal*cos(poseAug(3)+angular_dis)/2/odoB_kf;
      1/odoB_kf -1/odoB_kf];
             
nf = size(poseAug,1) - 3;

if nf > 0
    FFx = [Fx zeros(3,nf); ...
       zeros(nf,3) eye(nf,nf)];
   
    FFu = [Fu;zeros(nf,2)];
else
    FFx = Fx;
    FFu = Fu;
end
poseAug(1:3) = poseOut;
poseCovAug = FFx*poseCovAug*FFx'+FFu*[kR_kf*abs(delSr) 0;0 kL_kf*abs(delSl)]*FFu';


%% Covariance update
% Fx = [0 0 -sum_dl_dr_hal*sin(pose(3)+angular_dis);
%       0 0  sum_dl_dr_hal*cos(pose(3)+angular_dis);
%       0 0 0];

% Pxx = poseCovAug(1:3,1:3);
% if size(poseCovAug,1) > size(pose)
%     Pxm = poseCovAug(1:3,4:end);
%     Pmx = poseCovAug(4:end,1:3);
%     Pmm = poseCovAug(4:end,4:end);
    
    %% copy
%     poseAug(1:3) = pose;
%     poseCovAug(1:3,1:3) = poseCov;
    
    %% map and state covariance part update
%     Pxm = Pxm + Fx*Pxm;
%     Pmx = Pmx + Pmx*Fx';
    
%     poseCovAug(1:3,4:end) = Pxm;
%     poseCovAug(4:end,1:3) = Pmx;
    
% else
    %% copy
%     poseAug(1:3) = pose;
%     poseCovAug(1:3,1:3) = poseCov;
% end

end

