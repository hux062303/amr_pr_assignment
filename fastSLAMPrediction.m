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

    for i = 1:1:size(particles,2)    
        delSrRNG = delSr + normrnd(0,sqrt(Pin(1,1)));
        delSlRNG = delSl + normrnd(0,sqrt(Pin(2,2)));

        sum_dl_dr = delSrRNG + delSlRNG;
        sum_dl_dr_hal = sum_dl_dr * 0.5;
        sub_dr_dl = delSrRNG - delSlRNG;
        angular_dis = sub_dr_dl * 0.5 / odoB_kf;

        posePred = A * particles(:,i) + [sum_dl_dr_hal * cos(particles(3,i)+angular_dis);
                        sum_dl_dr_hal * sin(particles(3,i)+angular_dis);
                        angular_dis * 2];

        posePred(3) = atan2(sin(posePred(3)),cos(posePred(3)));

        particles(:,i) = posePred;

        poseOut = poseOut + weights(i).*posePred; 
        cosSum = weights(i).*cos(posePred(3));
        sinSum = weights(i).*sin(posePred(3));
    end

    poseOut(3) = atan2(sinSum, cosSum);

    covOut = zeros(size(particles,1),size(particles,1));

    for i = 1:1:size(particles,2)
        err = particles(:,i) - poseOut;
        err(3) = computeAngleDiff(poseOut(3), particles(3,i)); 
        covOut = covOut + weights(i).*(err)*(err)';
    end

end
