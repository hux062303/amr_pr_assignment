function [Y, SLAM_FEATURE_ID, weights]= fastSLAMAddNewFeatures(rt_vision, idEst, X, Y, SLAM_FEATURE_ID, weights)
    %% verify if new features existed
    if isempty(idEst)
        return;
    end

    dimC = 0;

    %% how many new features been measured at current frame
    measurement = [];
    for i = 1:1:size(idEst,1)
        isTracked = ismember(SLAM_FEATURE_ID, idEst(i));
        if isempty(find(isTracked)) || isempty(SLAM_FEATURE_ID)   % new feature
           SLAM_FEATURE_ID = [SLAM_FEATURE_ID;idEst(i)];
           dimC = dimC + 1; 
           measurement = [measurement;rt_vision(i,:)];
        end
    end

    if dimC == 0
        return;
    end

    global varVision
    Rs = [varVision 0 0;0 varVision 0;0 0 varVision*2];%% initlization

%     cosSum = 0;sinSum = 0;
%     poseAvg = zeros(3,1);
%     for i = 1:1:size(X,2)    
%         posePred = X(:,i);
%         poseAvg = poseAvg + weights(i).*posePred; 
%         cosSum = weights(i).*cos(posePred(3));
%         sinSum = weights(i).*sin(posePred(3));
%     end
%     poseAvg(3) = atan2(sinSum, cosSum);
    
    Rs = zeros(3,3,dimC);
    ts = zeros(3,dimC);
    for j = 1:dimC
        rt = measurement(j,:)';
        R = so3_exp(rt(1:3));
        t = rt(4:6);
        Rs(:,:,j) = R;
        ts(:,j) = t;
    end
    R = zeros(3,3);

    for j = 1:size(Y,1)
        pose = Y{j}.pose;
        %% construct
        vehicleYaw = pose(3);
        cy = cos(vehicleYaw);
        sy = sin(vehicleYaw);
        
%         covOut = zeros(3,3);
%         err = pose - poseAvg;
%         covOut = (err)*(err)';
        
        for i = 1:1:dimC    
            %% get vision relative measurement
%             rt = measurement(i,:)';
            R = Rs(:,:,i);
            t = ts(:,i);

            %% since those are rotation from camera to marker
            global camRelPos camRelRot

            Rm2b = R * camRelRot;% from body to marker
            Rb2g = [cy sy 0;-sy cy 0;0 0 1];% from ground to body
            Rm2g = Rm2b * Rb2g;% from ground to marker
            [yawFromG, pitchFromG, rollFromG] = dcm2angle(Rm2g);% decompose

            markerYaw = atan2(sin(yawFromG),cos(yawFromG));% round
   
            Rg2b = [cy -sy 0;sy cy 0;0 0 1];% from body to ground
            tg = Rg2b * (camRelRot' * (t - camRelPos'));% first, project translation from camera to body, then to ground
            
            % add to marker's ground position
            markerX = pose(1) + tg(1);
            markerY = pose(2) + tg(2);

            Y{j}.means = [Y{j}.means;{markerX markerY markerYaw}];
            
            Fu = [cy -sy 0;sy cy 0;0 0 1];
            Fx = eye(3,3) + [0 0 -sy*tg(1)-cy*tg(2);0 0 cy*tg(1)-sy*tg(2);0 0 0];
            Qt = Fu*Rs*Fu';%Fx*covOut*Fx' + 
            
            Y{j}.covs = [Y{j}.covs;{Qt}];
        end
    end
%     M = size(Y,1);
%     weights = ones(M,1)./M;
end

