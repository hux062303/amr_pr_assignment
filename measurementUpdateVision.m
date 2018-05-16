function [ poseOut, poseCovOut ] = measurementUpdateVision( poseIn, poseCovIn, estPose )
%[ poseOut, poseCovOut ] =MEASUREMENTUPDATE ( poseIn, poseCovIn,
    % Constants
    % The laser scanner pose in the robot frame is read globally(lsrRelpose)
    % The varAlpha and varR are the assumed variances of the parameters of
    % the extracted lines, they are also read globally
    global varVision

    poseOut = poseIn;
    poseCovOut = poseCovIn;

    %% serial processing
    for i = 1:1:size(estPose,1)% traverse all matches
        scale = estPose(i,4);
%         scale
        R = [varVision 0 0;0 varVision 0;0 0 varVision*2];

        C = [1 0 0;0 1 0;0 0 1];
        sigma_in = C*poseCovOut*C';
        K = poseCovOut*C'*inv(sigma_in+R);
        
        visionYaw = estPose(i,3);
        predictYaw = poseOut(3);
%         dYaw = visionYaw - predictYaw;
%         if dYaw < -pi 
%             dYaw = 2*pi + dYaw;
%         elseif dYaw > pi
%             dYaw = -2*pi + dYaw;
%         end
        dYaw = computeAngleDiff(predictYaw, visionYaw);
        residual = [estPose(i,1:2)' - poseOut(1:2);dYaw];
        poseOut = poseOut + K*(residual);
        poseOut(3) = atan2(sin(poseOut(3)),cos(poseOut(3)));
        
        poseCovOut = poseCovOut - K*sigma_in*K';
    end
end
