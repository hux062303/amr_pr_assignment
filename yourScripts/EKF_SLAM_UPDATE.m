function [poseAug, poseCovAug]= EKF_SLAM_UPDATE(se3_vision, idEst, poseAug, poseCovAug, SLAM_FEATURE_ID)
%UNTITLED11 ????????????????????????
%   ????????????????

%% verify if current estimation already in SLAM tracked features
if isempty(SLAM_FEATURE_ID)
    return;
end

dimC = 0;

%% how many features been measured at current frame
offset = [];
measurement = [];
for i = 1:1:size(idEst,1)
    isTracked = ismember(SLAM_FEATURE_ID, idEst(i));
    if ~isempty(find(isTracked))
       dimC = dimC + 1; 
       offset = [offset;find(isTracked,1)];
       measurement = [measurement;se3_vision(i,:)];
    end
end

global varVision
Rs = [varVision 0 0;0 varVision 0;0 0 varVision*2];

nx = size(poseAug,1);
C = zeros(dimC*3,nx);
residuals = zeros(dimC*3,1);
RR = zeros(dimC*3,dimC*3);

% poseAug(1:3,:) = pose;
% poseCovAug(1:3,1:3) = poseCov;

%% construct measurement matrix and innovations
vehicleYaw = poseAug(3);
cy = cos(vehicleYaw);
sy = sin(vehicleYaw);

if dimC == 0
    return;
end

for i = 1:1:dimC
    index = (i-1)*3;
    indexm = (offset(i)-1)*3+3;
    
    C(index+1,1) = -cy;     C(index+1,indexm+1) = cy;
    C(index+1,2) = -sy;     C(index+1,indexm+2) = sy;
    C(index+1,3) = (-sy*poseAug(indexm+1)+cy*poseAug(indexm+2))-(-sy*poseAug(1)+cy*poseAug(2));
    
    C(index+2,1) = sy;     C(index+2,indexm+1) = -sy;
    C(index+2,2) = -cy;     C(index+2,indexm+2) = cy;
    C(index+2,3) = (-cy*poseAug(indexm+1)-sy*poseAug(indexm+2))-(-cy*poseAug(1)-sy*poseAug(2));
    
    C(index+3,3) = -1;C(index+3,indexm+3) = 1;
    
    %% get vision relative measurement
    se3 = measurement(i,:)';
    R = so3_exp(se3(1:3));
    t = se3(4:6);
    
    %% since those are rotation from camera to marker
    global camRelPos camRelRot
    
    Rm2g = R*camRelRot*[cy sy 0;-sy cy 0;0 0 1];
    [yawMarker, pitchMarker, rollMarker] = dcm2angle(Rm2g);
    
    Rm2b = R * camRelRot;
    [yawFromBody, pitchFromBody, rollFromBody] = dcm2angle(Rm2b);

    Rg2b = [cy sy;-sy cy];
%     tg = Rg2b * (camRelRot' * t - camRelPos);
    dtGlobal = [poseAug(indexm+1) - poseAug(1);poseAug(indexm+2) - poseAug(2)];
    dtBody = Rg2b * dtGlobal;
%     dtCam = camRelRot * [dtBody;0] + camRelPos';
    
    tbody = camRelRot'*(t - camRelPos');

    %% dx dy 
    residuals(index+1,1) = tbody(1) - dtBody(1);
    residuals(index+2,1) = tbody(2) - dtBody(2);
            
    %% dyaw
    markerYaw = poseAug(indexm+3);
%     dYaw = markerYaw - vehicleYaw;
%     if dYaw < -pi 
%         dYaw = 2*pi + dYaw;
%     elseif dYaw > pi
%         dYaw = -2*pi + dYaw;
%     end
    dYaw = computeAngleDiff(vehicleYaw, markerYaw);
    
    residuals(index+3,1) = yawFromBody - dYaw;

%     residuals(index+3,1) = computeAngleDiff(dYaw, yawFromBody);
    
    RR(index+1:index+3,index+1:index+3) = Rs;
end

sigma_in = C*poseCovAug*C';
K = poseCovAug*C'*inv(sigma_in+RR);

poseAug = poseAug + K*(residuals);
poseAug(3) = atan2(sin(poseAug(3)),cos(poseAug(3)));
% poseCovAug = poseCovAug - K*sigma_in*K';
poseCovAug = (eye(nx,nx) - K*C)*poseCovAug*(eye(nx,nx) - K*C)'+K*RR*K';

% pose = poseAug(1:3);
% pose(3) = atan2(sin(pose(3)),cos(pose(3)));
% poseCov = poseCovAug(1:3,1:3);

end

