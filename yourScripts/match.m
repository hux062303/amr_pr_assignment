function [ matchResult ] = match( pose, poseCov, worldLines, laserLines )
% [matchResult] = MATCH(pose,poseCov,worldLines,laserLines)
%   This function matches the predicted lines to the extracted lines. The
%   criterion for a match is the mahalanobis distance between the (alpha,
%   r) parameters of the predicted and the extracted lines. The arguments
%   are:
%       pose: The estimated robot pose given as [x,y,theta]
%       poseCov: The estimated covariance matrix of the robot pose
%       worldLines: Known world lines in world coordinates, given as
%       [alpha;r] for each line. Number of rows = number of lines
%       laserLines: Lines extracted from the laser scan. Given as [alpha;r]
%       for each line. Number of rows = number of lines
%
%       matchResult: A (5xnoOfWorldLines) matrix whose columns are 
%       individual pairs of line matches. It is structured as follows:
%       matchResult = [ worldLine(1,1) , worldLine(1,2) ...  ]
%                     [ worldLine(2,1) , worldLine(2,2)      ]
%                     [ innovation1(1) , innovation2(1)      ]
%                     [ innovation1(2) , innovation2(2)      ]
%                     [ matchIndex1    , matchIndex2    ...  ]
%           Note that the worldLines are in the world coordinates!

    % The varAlpha and varR are the assumed variances of the parameters of
    % the extracted lines, they are read globally.
    global varAlpha varR lsrRelPose
g = 2;
matchResult = zeros(5,size(worldLines,2));
% R = [varAlpha 0;0 varR];
for i = 1:1:size(worldLines,2)
%     a = worldLines(1,i);
%     C = [0 0 -1;...
%     -cos(a) -sin(a) ...
%     -cos(a).*(-sin(pose(3)).*lsrRelPose(1)-cos(pose(3)).*lsrRelPose(2))...
%     -sin(a).*(cos(pose(3)).*lsrRelPose(1)-sin(pose(3)).*lsrRelPose(2))];
%     lineCov = C*poseCov*C' + R;
    [ projectedLine, lineCov ] = projectToLaser( worldLines(:,i),pose, poseCov);
    
    %% compute Mahalanobis distance
    err = zeros(2,size(laserLines,2));
    err(1,:) = -projectedLine(1) + laserLines(1,:);
    err(2,:) = -projectedLine(2) + laserLines(2,:);
    distMahalanomis = err'*inv(lineCov)*err;
%     id_matched = distMahalanomis < g^2;
    distMahalanomis = diag(distMahalanomis);
    [val, id] = min(distMahalanomis);
    if distMahalanomis(id) < g^2
        matchResult(1:2,i) = worldLines(1:2,i);
        matchResult(3:4,i) = err(1:2,id);
        matchResult(5,i) = id;
    end
end

end
