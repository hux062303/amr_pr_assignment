function [ poseOut, poseCovOut ] = measurementUpdate( poseIn, poseCovIn, matchResult )
%[ poseOut, poseCovOut ] =MEASUREMENTUPDATE ( poseIn, poseCovIn,
%matchResult ) perform extended Kalman filter measurement update on the
%estimated robot pose poseIn with covariance poseCovIn using a set of
%matched predicted and extracted laser scanner lines given in matchResult.
%The arguments are defined as:
%       poseIn: The estimated robot pose given as [x,y,theta]
%       poseCovIn: The estimated covariance matrix of the robot pose
%       matchResult: A (5xnoOfWorldLines) matrix whose columns are 
%       individual pairs of line matches. It is structured as follows:
%       matchResult = [ worldLine(1,1) , worldLine(1,2) ...  ]
%                     [ worldLine(2,1) , worldLine(2,2)      ]
%                     [ innovation1(1) , innovation2(1)      ]
%                     [ innovation1(2) , innovation2(2)      ]
%                     [ matchIndex1    , matchIndex2    ...  ]
%           Note that the worldLines are in the world coordinates!
%       
%       poseOut: The updated robot pose estimate
%       poseCovOut: The updated estimate of the robot pose covariance 
%       matrix 

    % Constants
    % The laser scanner pose in the robot frame is read globally(lsrRelpose)
    % The varAlpha and varR are the assumed variances of the parameters of
    % the extracted lines, they are also read globally
    global lsrRelPose varAlpha varR

    poseOut = poseIn;
    poseCovOut = poseCovIn;
    R = [varAlpha 0;0 varR];

    %% serial processing
    for i = 1:1:size(matchResult,2)% traverse all matches
        if matchResult(5,i)~=0 % if matched
            a = matchResult(1,i);
            C = [0 0 -1;...
                -cos(a) -sin(a) ...
                -cos(a).*(-sin(poseIn(3)).*lsrRelPose(1)-cos(poseIn(3)).*lsrRelPose(2))-sin(a).*(cos(poseIn(3)).*lsrRelPose(1)-sin(poseIn(3)).*lsrRelPose(2))];
            % new kalman gain
            sigma_in = C*poseCovOut*C';
            K = poseCovOut*C'*inv(sigma_in+R);
            % apply innovation
            v = matchResult(3:4,i);
            poseOut = poseOut + K*v;
            % update covariance
            poseCovOut = poseCovOut - K*sigma_in*K';
        end
    end
end
