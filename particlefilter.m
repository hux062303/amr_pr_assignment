clear all
close all
addpath([pwd filesep 'extras'])
addpath([pwd filesep 'yourScripts'])
addpath([pwd filesep 'extras' filesep 'arrow'])
addpath([pwd filesep 'extras' filesep 'gaussianEllipsoid'])

global realPose

%% Constants
constants % Calling the script with the constants

%% Run
realPose = pose + randn(3,1).*sqrt([poseCov(1,1); poseCov(2,2); poseCov(3,3)]); %Generate a real pose based on the initial uncertainty

simulateWorld(0); % Initialize simulateWorld

%Generate the arrays for logging
poses = zeros(3,noOfIter);
poseCovs = cell(1,noOfIter);
realPoses = zeros(3,noOfIter);

useLaser = 0;
useCamera = 1;
doEKF_SLAM = 0;
SLAM_FEATURE_ID = [];
poseAug = [];
poseCovAug = [];

% posesSLAM_Features = zeros(3,noOfIter);
% poseCovs = cell(1,noOfIter);

%% particle filter parameters
M = 1000;
X = zeros(numel(pose),M);
weights = ones(M,1);
weights = weights ./ M;
P = eye(numel(pose)).*0;
%% sampling
for i = 1:1:M
    X(:,i) = pose + normrnd(zeros(numel(pose),1),sqrt(diag(P)),[numel(pose),1]);
end

for iter = 1:noOfIter
    % Move a bit
    [delSr, delSl]=simulateWorld(iter); %delSr and delSl are the left and right wheel displacements

    %Your script performing position prediction
%     [pose, poseCov] = positionPrediction(pose, poseCov, delSr, delSl);
    % do particle prediction here
    [pose, poseCov, X] = particlePrediction(pose, poseCov, delSr, delSl, X, weights);

    poses(:,iter) = pose;
    poseCovs{1,iter} = poseCov;
    realPoses(:,iter) = realPose;
    
    %Plot the real path along with the predicted path
    if(mod(iter,plotNth) == 0 || mod(iter,lsrPer) == 0)
        figure(1)
        handles = [];
        handles(1)=plot(poses(1,1:iter)',poses(2,1:iter)','r');
        hold on
        handles(2)=plot(realPoses(1,1:iter)',realPoses(2,1:iter)','b');
        plot(worldPoints(1,:)',worldPoints(2,:),'k-');    
        arrow(pose(1:2),pose(1:2)+[cos(pose(3));sin(pose(3))]/5,10,[],[],[],'FaceColor','r');
        arrow(realPose(1:2),realPose(1:2)+[cos(realPose(3));sin(realPose(3))]/5,10,[],[],[],'FaceColor','b');
        for ellipsoidIndex = [1:plotNthEllipsoid:iter iter]
            h=plot_gaussian_ellipsoid(poses(1:2,ellipsoidIndex), poseCovs{1,ellipsoidIndex}(1:2,1:2));
            set(h,'color','r');
            plot(poses(1,ellipsoidIndex),poses(2,ellipsoidIndex),'rx')
        end
        title('Real and the estimated paths of the robot')
        xlabel('x(m)');
        ylabel('y(m)');
        legend(handles, 'Estimated path','Actual path');
        hold off
        pause(0.01)
    end
    
    %%
    if(mod(iter,camPer) == 0 && useCamera == 1)
        [im, corners, id] = fakeImage(realPose, camRelPos, camRelRot, cameraMatrix, distorsionCoeff, markerPosition); 
      
        %% image processing and OCR recognition
        [idEst, cornersEst] = imgRecognition(im,cameraMatrix,distorsionCoeff);

        if isempty(find(ismember(idEst, id)))
            idEst = [];
        end
        
        if ~(isempty(idEst)) && idEst >=1 && idEst <= 4
            [se3_vision,reproj_error] = solvePnP(cornersEst, idEst, cameraMatrix, distorsionCoeff);
            %% solve PnP to get relative pose
            estPose = getPoseFromIm(idEst, se3_vision, camRelPos, camRelRot, markerPosition);
            %% particle filter
%                [pose, poseCov] = measurementUpdateVision(pose,poseCov, estPose);
           [pose, poseCov, X, weights] = particlesUpdateVision(pose, poseCov, estPose, X, weights);
        end

        %% plot
%         figure(3)
%         gim = imshow(im,[]);hold on;
%         for ii = 1:4:size(corners,1)
%             plot([corners(ii,2) corners(ii+1,2)],[corners(ii,1) corners(ii+1,1)],'b-');
%             plot([corners(ii+1,2) corners(ii+2,2)],[corners(ii+1,1) corners(ii+2,1)]);
%             plot([corners(ii+2,2) corners(ii+3,2)],[corners(ii+2,1) corners(ii+3,1)]);
%             plot([corners(ii+3,2) corners(ii,2)],[corners(ii+3,1) corners(ii,1)]);
%         end
        
        %% plot tracked features
        figure(1);%%hold on;
        if (mod(iter, plotNthSLAMFeatures) == 0)
            for ii = 1:1:size(SLAM_FEATURE_ID,1)
                pose_features = poseAug(3+(ii-1)*3+1:3+(ii-1)*3+2);
                poseCov_features = poseCovAug(3+(ii-1)*3+1:3+(ii-1)*3+2,3+(ii-1)*3+1:3+(ii-1)*3+2);
                h=plot_gaussian_ellipsoid(pose_features, poseCov_features);
                set(h,'color','m');
                plot(pose_features(1),pose_features(2),'m+');
            end
        end
    end
end
