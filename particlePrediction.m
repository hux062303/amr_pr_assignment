function [ poseOut, covOut, particles] = particlePrediction( poseIn,covIn,delSr,delSl, particles, weights)
%[poseOut, covOut] = POSITIONPREDICTION(poseIn,covIn,delSr,delSl) perform
%one step of robot pose prediction from a set of wheel displacements
%   poseIn = old robot pose
%   covIn = uncertainty on the old robot pose
%   delSr = right wheel linear displacement
%   delSl = left wheel linear displacement

%% Constants
% The robot parameters are read globally, odoB is the wheel separation, kR
% and kL are the odometry uncertainty parameters
global odoB kR kL 
global odoB_kf kR_kf kL_kf

%% pose update
Pin = [kR_kf*abs(delSr) 0;0 kL_kf*abs(delSl)];

% poseOut = [0;0;0];
A = eye(3,3);

poseOut = zeros(numel(poseIn),1);
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
%     if (posePred(3) > pi)
%         posePred(3) = posePred(3) - 2*pi;
%     elseif posePred(3)<-pi
%         posePred(3) = posePred(3) + 2*pi;
%     end
    
    particles(:,i) = posePred;
    
%     if abs(abs(posePred(3))-pi) < 0.3 %% around pi
%         if posePred(3) < 0
%             posePred(3) = posePred(3) + 2*pi;
%         end
%     end
    
    poseOut = poseOut + weights(i).*posePred; 
    cosSum = weights(i).*cos(posePred(3));
    sinSum = weights(i).*sin(posePred(3));
end

poseOut(3) = atan2(sinSum, cosSum);

% if (poseOut(3) > pi)
%     poseOut(3) = poseOut(3) - 2*pi;
% elseif poseOut(3)<-pi
%     poseOut(3) = poseOut(3) + 2*pi;
% end

covOut = zeros(numel(poseIn),numel(poseIn));

for i = 1:1:size(particles,2)
    err = particles(:,i) - poseOut;
    err(3) = computeAngleDiff(poseOut(3), particles(3,i)); 
%     acos(sin(particles(3,i))*sin(poseOut(3))+cos(particles(3,i))*cos(poseOut(3)));
    covOut = covOut + weights(i).*(err)*(err)';
end

end
