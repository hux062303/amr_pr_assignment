function [poseAug, poseCovAug, SLAM_FEATURE_ID]= EKF_SLAM_ADD_NEW_FEATURES(se3_vision, idEst, poseAug, poseCovAug, SLAM_FEATURE_ID)
%UNTITLED12 ????????????????????????
%   ????????????????

%% verify if new features existed
if isempty(idEst)
    return;
end

% if isempty(poseAug)
%     poseAug = pose;
%     poseCovAug = poseCov;
%     SLAM_FEATURE_ID = [];
% end

dimC = 0;

%% how many new features been measured at current frame
measurement = [];
for i = 1:1:size(idEst,1)
    isTracked = ismember(SLAM_FEATURE_ID, idEst(i));
    if isempty(find(isTracked)) || isempty(SLAM_FEATURE_ID)   % new feature
       SLAM_FEATURE_ID = [SLAM_FEATURE_ID;idEst(i)];
       dimC = dimC + 1; 
       measurement = [measurement;se3_vision(i,:)];
    end
end

if dimC == 0
    return;
end

global varVision

Rs = [varVision*1 0 0;0 varVision*1 0;0 0 varVision*2];

% poseAug(1:3,:) = pose;
% poseCovAug(1:3,1:3) = poseCov;
poseCov = poseCovAug(1:3,1:3);

%% construct
vehicleYaw = poseAug(3);
cy = cos(vehicleYaw);
sy = sin(vehicleYaw);

for i = 1:1:dimC    
    %% get vision relative measurement
    se3 = measurement(i,:)';
    R = so3_exp(se3(1:3));
    t = se3(4:6);
    
    %% since those are rotation from camera to marker
    global camRelPos camRelRot
    
    Rm2b = R * camRelRot;
%     [yawFromBody, pitchFromBody, rollFromBody] = dcm2angle(Rm2b);
%     markerYaw = yawFromBody + vehicleYaw;
    Rb2g = [cy sy 0;-sy cy 0;0 0 1];
    Rm2g = Rm2b * Rb2g;
    [yawFromG, pitchFromG, rollFromG] = dcm2angle(Rm2g);

    markerYaw = atan2(sin(yawFromG),cos(yawFromG));
%     if (markerYaw > pi)
%         markerYaw = markerYaw - 2*pi;
%     elseif markerYaw <-pi
%         markerYaw = markerYaw + 2*pi;
%     end    
    
    Rg2b = [cy -sy 0;sy cy 0;0 0 1];
    tg = Rg2b * (camRelRot' * (t - camRelPos'));
%     tg = Rg2b*camRelRot'*(R'*(-t) - camRelPos');
    
    markerX = poseAug(1) + tg(1);
    markerY = poseAug(2) + tg(2);

    poseAug = [poseAug;[markerX markerY markerYaw]'];
    
    Fx = eye(3,3) + [0 0 -sy*tg(1)-cy*tg(2);0 0 cy*tg(1)-sy*tg(2);0 0 0];
    Fu = [cy -sy 0;sy cy 0;0 0 1];
    
    nx = size(poseCovAug,1);

    poseCovAug = [poseCovAug zeros(nx,3); ...
                  zeros(3,nx) zeros(3,3)];
%     poseCovAug(1:3, nx+1:nx+3) = Fx*poseCov*Fx';
%     poseCovAug(nx+1:nx+3, 1:3) = Fx*poseCov*Fx';
    poseCovAug(nx+1:nx+3,nx+1:nx+3) = Fx*poseCov*Fx' + Fu*Rs*Fu';% 
end

end

