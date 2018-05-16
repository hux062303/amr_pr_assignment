function [ poseOut, covOut, particles] = fastSLAMPrediction(delSr,delSl, particles, weights)
%[poseOut, covOut] = POSITIONPREDICTION(poseIn,covIn,delSr,delSl) perform
%one step of robot pose prediction from a set of wheel displacements
%   poseIn = old robot pose
%   covIn = uncertainty on the old robot pose
%   delSr = right wheel linear displacement
%   delSl = left wheel linear displacement

    %% Constants
    % The robot parameters are read globally, odoB is the wheel separation, kR
    % and kL are the odometry uncertainty parameters
    global odoB_kf kR_kf kL_kf

    %% pose update
    Pin = [kR_kf*abs(delSr) 0;0 kL_kf*abs(delSl)];

    A = eye(3,3);
    poseOut = zeros(size(particles,1),1);
    cosSum = 0;
    sinSum = 0;
    
    M = size(particles,2);
    
    delSrRNG = delSr + normrnd(0,sqrt(Pin(1,1)),1,M);
    delSlRNG = delSl + normrnd(0,sqrt(Pin(2,2)),1,M);

    sum_dl_dr = delSrRNG + delSlRNG;
    sum_dl_dr_hal = sum_dl_dr * 0.5;
    sub_dr_dl = delSrRNG - delSlRNG;
    angular_dis = sub_dr_dl * 0.5 / odoB_kf;    
    
    posePred = A * particles + [sum_dl_dr_hal .* cos(particles(3,:)+angular_dis); ...
                                sum_dl_dr_hal .* sin(particles(3,:)+angular_dis); ...
                                angular_dis * 2];
    
    posePred(3,:) = atan2(sin(posePred(3,:)),cos(posePred(3,:)));
                  
    particles = posePred;
    poseOut = sum(repmat(weights',3,1).*posePred,2); 
    
    cosSum = sum((weights').*cos(posePred(3,:)),2);
    sinSum = sum((weights').*sin(posePred(3,:)),2);

    
%     for i = 1:1:size(particles,2)    
%         particles(:,i) = posePred;
% 
%         poseOut = poseOut + weights(i).*posePred; 
%         cosSum = weights(i).*cos(posePred(3));
%         sinSum = weights(i).*sin(posePred(3));
%     end
    
    scale = sqrt(1 / (cosSum*cosSum+sinSum*sinSum));
    cosSum = cosSum * scale;
    sinSum = sinSum * scale;

    poseOut(3) = atan2(sinSum, cosSum);

    covOut = zeros(size(particles,1),size(particles,1));
    
    poseOutRep = repmat(poseOut,1,M);
    errs = particles - poseOutRep;
    errs(3,:) = arrayfun(@computeAngleDiff,poseOutRep(3,:), particles(3,:));
    for i = 1:1:M   
        %err(3) = computeAngleDiff(poseOut(3), particles(3,i)); 
        err = errs(:,i);
        covOut = covOut + weights(i).*(err)*(err)';
    end

end
