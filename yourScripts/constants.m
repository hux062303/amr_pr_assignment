%% World constants
% The box
global lines
squareWidth = 1.84; % The width of the box that the robot is in
lines = [[1;1;1;-1],[1;-1;-1;-1],[-1;-1;-1;1],[-1;1;1;1]]*squareWidth/2; % Lines describing the box
worldPoints = [lines(1:2,:) lines(1:2,1)]; % Corner points of the box
noOfWorldLines = size(lines,2);
worldLines =convertToPolar(lines); % The lines in the (alpha, rho) parameterization

%% Real Robot constants
global odoB kR kL
odoB = 0.3; % The separation between the two wheels
kR = 0.001; % The variance of the linear uncertainty for a 1m move for the right wheel
kL = 0.001; % The variance of the linear uncertainty for a 1m move for the left wheel
smrNo=0
%% Kalman filter Robot constants
global odoB_kf kR_kf kL_kf
odoB_kf = 0.3; % The separation between the two wheels
kR_kf = 0.01; % The variance of the linear uncertainty for a 1m move for the right wheel
kL_kf = 0.01; % The variance of the linear uncertainty for a 1m move for the left wheel

%% Simulation constants
global ts robotLinearSpeed robotAngularSpeed robotPathWidth robotPathRadius trackNo
noOfIter = 8000; %The number of simulation iterations
ts = 0.01; %The time period of each iteration
plotNth = 10; %Plot the pose data every nth iteration
plotNthEllipsoid = 100; %Plot a gaussian ellipsoid for every nth pose
simulation=true;

%% for SLAM
plotNthSLAMFeatures = 100; %Plot a gaussian ellipsoid for every nth pose

%% Path constants
trackNo = 3; %The path followed by the robot, 1: line, 2: circle, 3: square
robotLinearSpeed = 0.3; %The linear speed of the robot for path segments that it is moving forward
robotAngularSpeed = 1; %The angular speed of the robot fot path segments that it is turning
robotPathWidth = squareWidth - 0.6; % For the square path, the width of the square
robotPathRadius = 1; % For the circular path, the radius of the circle

%% Laser scanner constants
global varAlpha varR
global lsrRelPose
lsrPer = 100; %The period of laser scans
lsrRelPose = [0.28,0,0]; %The pose of the laser scanner in the robot frame
lsrRelRot = [cos(lsrRelPose(3)) sin(lsrRelPose(3)); -sin(lsrRelPose(3)) cos(lsrRelPose(3))];
varAlpha = 0.001; %The assumed variance on the orientation of a line extracted from the laser scanner
varR = 0.0004; %%The assumed variance on the distance of a line extracted from the laser scanner

plotLaserDataWithPredictedLines = true;
plotExtractedLines = true;
plotLineParameters = false;

plotMatchedLines = false;
plotAfterMeasurementUpdate = false;


%% RANSAC constants
maxNoOfLines = 50; % The maximum number of line extraction iterations
noOfRandomCouples = 20; % The number of random point couples to try before choosing the best candidate
distThreshold=0.01; % The distance threshold determining whether a point is supporting a line
minLineSupport=20; % The minimum number of points to support a line for the line to be accepted
minNoOfPoints=20; % The minimum number of points to continue line extraction iterations


%% Initials
pose = [-squareWidth/2+0.1;-squareWidth/2+0.1;0]; % The !!estimated!! initial pose of the robot

poseCov = [0.0 0 0  % The assumed uncertainty on the initial pose
           0 0.0 0
           0 0 0.0];
       
%% camera intrisincs
fx = 1000;
fy = 1000;
skew = 0;
u0 = 320;
v0 = 240;
cameraMatrix = [fx skew u0; ...
                 0   fy v0; ...
                 0    0  1];
camPer = 0.1 / ts;
global camRelPos camRelRot
camRelPos = [0 0 0];
camRelRot = [0 -1 0;0 0 -1;1 0 0];
%% distorsion parameters, k1, k2, p1, p2, k3
distorsionCoeff = [0 0 0 0 0];

%% add marker
markercenterPosition = [[-0.55, -1, 0]; ...
                        [1,  -0.5, 0]; ... 
                        [0.6, 1, 0]; ...
                        [-1, 0.6, 0];];
markerSize = 0.1;
global hmarkerSize
hmarkerSize = markerSize * 0.5;
markerPosition = zeros(size(markercenterPosition,1),4,3);
%% 1st marker
markerPosition(1,1,:) = [markercenterPosition(1,1)+hmarkerSize, markercenterPosition(1,2), hmarkerSize];
markerPosition(1,2,:) = [markercenterPosition(1,1)-hmarkerSize, markercenterPosition(1,2), hmarkerSize];
markerPosition(1,3,:) = [markercenterPosition(1,1)-hmarkerSize, markercenterPosition(1,2), -hmarkerSize];
markerPosition(1,4,:) = [markercenterPosition(1,1)+hmarkerSize, markercenterPosition(1,2), -hmarkerSize];
%% 2nd
markerPosition(2,1,:) = [markercenterPosition(2,1), markercenterPosition(2,2)+hmarkerSize, hmarkerSize];
markerPosition(2,2,:) = [markercenterPosition(2,1), markercenterPosition(2,2)-hmarkerSize, hmarkerSize];
markerPosition(2,3,:) = [markercenterPosition(2,1), markercenterPosition(2,2)-hmarkerSize, -hmarkerSize];
markerPosition(2,4,:) = [markercenterPosition(2,1), markercenterPosition(2,2)+hmarkerSize, -hmarkerSize];
%% 3nd
markerPosition(3,1,:) = [markercenterPosition(3,1)-hmarkerSize, markercenterPosition(3,2), hmarkerSize];
markerPosition(3,2,:) = [markercenterPosition(3,1)+hmarkerSize, markercenterPosition(3,2), hmarkerSize];
markerPosition(3,3,:) = [markercenterPosition(3,1)+hmarkerSize, markercenterPosition(3,2), -hmarkerSize];
markerPosition(3,4,:) = [markercenterPosition(3,1)-hmarkerSize, markercenterPosition(3,2), -hmarkerSize];
%% 4th
markerPosition(4,1,:) = [markercenterPosition(4,1), markercenterPosition(4,2)-hmarkerSize, hmarkerSize];
markerPosition(4,2,:) = [markercenterPosition(4,1), markercenterPosition(4,2)+hmarkerSize, hmarkerSize];
markerPosition(4,3,:) = [markercenterPosition(4,1), markercenterPosition(4,2)+hmarkerSize, -hmarkerSize];
markerPosition(4,4,:) = [markercenterPosition(4,1), markercenterPosition(4,2)-hmarkerSize, -hmarkerSize];

global R33_Markers
R33_Markers = zeros(4,3,3);
R33_Markers(1,:,:) = [-1 0 0;0  0 1;0 1 0];
R33_Markers(2,:,:) = [ 0 -1 0;0 0 1;-1 0 0];
R33_Markers(3,:,:) = [1 0 0;0 0 1;0 -1 0];
R33_Markers(4,:,:) = [0 1 0;0 0 1;1 0 0];

global varVision
varVision = 0.005;

global idBasePath
idBasePath = '/digits/id/'

trainBasePath = '/digits/'
global OCRMDL
trainX = zeros(2000,20*20);
trainY = zeros(2000,1);
validTrainDataCnt = 0;
for i = 1:1:9
    trainingDataPath = strcat(fullfile(pwd),trainBasePath,int2str(i),'/');
    imgFiles = dir(fullfile(trainingDataPath,'*.jpg'));
    imgFileNames = {imgFiles.name}';
    
    for j = 1:1:size(imgFileNames,1)
        validTrainDataCnt = validTrainDataCnt + 1;
        traingImg = imread(strcat(trainingDataPath,imgFileNames{j}));        
        traingImg = imresize(traingImg,[20,20]);
        traingImg(traingImg>80) = 255;
        traingImg(traingImg<=80) = 0;
        trainX(validTrainDataCnt,:) = traingImg(:)';
        trainY(validTrainDataCnt,1) = i;
    end
end

trainX = trainX(1:validTrainDataCnt,:);
trainY = trainY(1:validTrainDataCnt,:);
OCRMDL = fitcknn(trainX,trainY,'NumNeighbors',10);