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

featurePoses = [];
featureCovs = [];

useCamera = 1;
doFASTSLAM = 1;
SLAM_FEATURE_ID = [];

%% particle filter parameters
M = 500;
weights = ones(M,1);
weights = weights ./ M;

X = zeros(numel(pose),M);% pose particles
Y = cell(M,1);
P = eye(numel(pose)).*0;

%% sampling
for i = 1:1:M
    X(:,i) = pose + normrnd(zeros(numel(pose),1),sqrt(diag(P)),[numel(pose),1]);
    Y{i}.pose = X(:,i);
    Y{i}.means = cell(0);
    Y{i}.covs = cell(0);
end

% Y.X = X;
frames=[];

for iter = 1:noOfIter
    % Move a bit
    [delSr, delSl]=simulateWorld(iter); %delSr and delSl are the left and right wheel displacements
    
    % retrieve pose
    if doFASTSLAM == 1
        for j = 1:1:M
            X(:,j) = Y{j}.pose;
        end
%         X = Y.X;
%         tic
%         Xc = cellfun(@(Yj)(Yj.pose),Y,'UniformOutput',false);
%         X = cell2mat(Xc');
%         toc
    end
    % sample pose
    [pose, poseCov, X] = fastSLAMPrediction(delSr, delSl, X, weights);        
%     Y.X = X;
%     tic
    for j = 1:1:M
        Y{j}.pose = X(:,j);
    end
%     toc
    
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
        
%         plotIdx = round(linspace(1, iter, 100));
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
    
    %% fastSLAM    
    if(mod(iter,camPer) == 0 && useCamera == 1)
        [im, corners, id] = fakeImage(realPose, camRelPos, camRelRot, cameraMatrix, distorsionCoeff, markerPosition); 
        %% image processing and OCR recognition
        [idEst, cornersEst] = imgRecognition(im,cameraMatrix,distorsionCoeff);
        if isempty(find(ismember(idEst, id)))
            idEst = [];
        end
        
        if ~(isempty(idEst))
            [rt_vision,reproj_error] = solvePnP(cornersEst, id, cameraMatrix, distorsionCoeff);
            if (doFASTSLAM == 0)
                %% solve PnP to get relative pose
                estPose = getPoseFromIm(idEst, rt_vision, camRelPos, camRelRot, markerPosition);
                %% particle filter
               [pose, poseCov, X, weights] = particlesUpdateVision(pose, poseCov, estPose, X, weights);
            else
                %% SLAM Update
                [X, Y, weights, SLAM_FEATURE_ID] = fastSLAMUpdate(rt_vision, idEst, X, Y, weights, SLAM_FEATURE_ID);
                %% augment state with mapped features
                [Y, SLAM_FEATURE_ID, weights]= fastSLAMAddNewFeatures(rt_vision, idEst, X, Y, SLAM_FEATURE_ID, weights);
            end
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

        [featurePoses, featureCovs] = computeFeaturePoseAndCov(Y, weights, SLAM_FEATURE_ID);
        
        %% plot tracked features
        figure(1);%%hold on;
        if (mod(iter, plotNthSLAMFeatures) == 0)
            for ii = 1:1:size(SLAM_FEATURE_ID,1)
                pose_features = featurePoses(1:2,ii);
                poseCov_features = zeros(3,3);
                poseCov_features = featureCovs(1:2,1:2,ii);
                h=plot_gaussian_ellipsoid(pose_features, poseCov_features);
                set(h,'color','m');
                plot(pose_features(1),pose_features(2),'m+');
            end
        end
    end
    frame=getframe;
    frames = [frames;frame];
end

v = VideoWriter('/Users/xiaohu/Downloads/ekf_slam/kalmanexercise/fastslam.avi');
open(v);
for i = 1:1:size(frames,1)
    writeVideo(v,frames(i));
end
close(v);

