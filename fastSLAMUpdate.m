function [X, Y, weights, SLAM_FEATURE_ID] = fastSLAMUpdate(rt_vision, idEst, X, Y, weights, SLAM_FEATURE_ID)
    
    doUpdate = 1;
    %% no estimation, no need to do update
    if isempty(idEst)
        doUpdate = 0;
    end

    %% verify if current estimation already in SLAM tracked features
    if isempty(SLAM_FEATURE_ID)
        doUpdate = 0;
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
           measurement = [measurement;rt_vision(i,:)];
        end
    end
    if dimC == 0
        doUpdate = 0;
    end
 
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
    if doUpdate == 1
        global varVision
        Rs = [varVision 0 0;0 varVision 0;0 0 varVision*2];
        
        for i = 1:size(Y,1)
            C = zeros(3,3);
            residuals = zeros(3,1);
            pose = Y{i}.pose;

            vehicleYaw = pose(3);
            cy = cos(vehicleYaw);
            sy = sin(vehicleYaw);
            for j = 1:1:dimC
                featurepose = [Y{i}.means{offset(j),:}]';
                featurecov = [Y{i}.covs{offset(j),:}];
                C(1,1) = cy; C(1,2) = sy;
                C(2,1) = -sy; C(2,2) = cy;
                C(3,3) = 1;
                %% get vision relative measurement
                R = Rs(:,:,j);
                t = ts(:,j);
                
                %% since those are rotation from camera to marker
                global camRelPos camRelRot
                Rm2b = R * camRelRot;
                [yawFromBody, pitchFromBody, rollFromBody] = dcm2angle(Rm2b);
                Rg2b = [cy sy;-sy cy];
                dtGlobal = [featurepose(1) - pose(1);featurepose(2) - pose(2)];
                dtBody = Rg2b * dtGlobal;
                tbody = camRelRot'*(t - camRelPos');
                %% dx dy 
                residuals(1,1) = tbody(1) - dtBody(1);
                residuals(2,1) = tbody(2) - dtBody(2);
                %% dyaw
                markerYaw = featurepose(3);
                dYaw = computeAngleDiff(vehicleYaw, markerYaw);
                residuals(3,1) = yawFromBody - dYaw;
                
                sigma_in = C*featurecov*C';
                K = featurecov*C'*inv(sigma_in+Rs);

                featurepose = featurepose + K*(residuals);
                featurepose(3) = atan2(sin(featurepose(3)),cos(featurepose(3)));
                featurecov = (eye(3)-K*C)*featurecov;

                Y{i}.means{offset(j),1} = featurepose(1);
                Y{i}.means{offset(j),2} = featurepose(2);
                Y{i}.means{offset(j),3} = featurepose(3);
                Y{i}.covs{offset(j),:} = featurecov;

                weights(i) = weights(i) * 1/sqrt((2*pi)^3*det(sigma_in+Rs))*exp(-0.5*residuals'*inv(sigma_in+Rs)*residuals);%weights(i) * 
            end
        end
        weights = weights ./ sum(weights);
    end
    
    %% resampling
    M = size(Y,1);
    Neff = 1/sum(weights.^2);
    resample_percentage = 0.80;
    Nt = resample_percentage*M;
    if Neff < Nt
        % resampling
        if true
            Ytemp = Y;
            rr = 1/M*rand(1);
            c = weights(1);
            i = 1;
            U = 0;
            step = 1/M;
            for m = 1:M
                U = rr + (m-1)*step;
                while U>c
                    i = i + 1;
                    c = c + weights(i);
                end
                Ytemp{m} = Y{i};
            end
            Y = Ytemp;
            for j = 1:1:M
                X(:,j) = Y{j}.pose;
            end
            weights = ones(M,1)./M;
        end
    end
end

