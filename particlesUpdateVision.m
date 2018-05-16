function [ poseOut, poseCovOut, particles, weights] = particlesUpdateVision( poseIn, poseCovIn, estPose, particles, weights)
%[ poseOut, poseCovOut ] =MEASUREMENTUPDATE ( poseIn, poseCovIn,
    % Constants
    % The laser scanner pose in the robot frame is read globally(lsrRelpose)
    % The varAlpha and varR are the assumed variances of the parameters of
    % the extracted lines, they are also read globally
    global varVision

    for i = 1:1:size(particles,2)
        %% get measure
        measPred = particles(1:3,i);
        measPred = repmat(measPred',size(estPose,1),1)';
        meas = estPose(1:3)';
        
        error = zeros(3,size(estPose,1));
        error = meas - measPred;
        %% dealing with yaw angle
        for j = 1:size(estPose,1)
            vYaw = meas(3,j);
            pYaw = measPred(3,j);
            
%             if vYaw < 0 
%                 vYaw = vYaw + 2*pi;
%             end

%             dYaw = vYaw - pYaw;
%             if dYaw < -pi 
%                 dYaw = 2*pi + dYaw;
%             elseif dYaw > pi
%                 dYaw = -2*pi + dYaw;
%             end
            error(3,j) = computeAngleDiff(vYaw, pYaw);
            
            %% compute likelihood
            R = [varVision 0 0;0 varVision 0;0 0 varVision*2];
            
            likelihood = 1/sqrt(det(2*pi.*R))*exp(-0.5*error'*inv(R)*error);
            
            weights(i) = weights(i)*likelihood;
            
        end 
    end
    weights = weights ./ sum(weights);

    poseOut = zeros(numel(poseIn),1);
    cosSum = 0;
    sinSum = 0;
    for i = 1:1:size(particles,2)
        poseUpdate = particles(:,i);
%         if (poseUpdate(3) < 0)
%             poseUpdate(3) = poseUpdate(3) + 2*pi;
%         end

%         if abs(abs(poseUpdate(3))-pi) < 0.3 %% around pi
%             if poseUpdate(3) < 0
%                 poseUpdate(3) = poseUpdate(3) + 2*pi;
%             end
%         end

        poseOut = poseOut + weights(i).*poseUpdate;
        
        cosSum = weights(i).*cos(poseUpdate(3));
        sinSum = weights(i).*sin(poseUpdate(3));
    end
    
%     if (poseOut(3) > pi)
%         poseOut(3) = poseOut(3) - 2*pi; 
%     elseif poseOut(3)<-pi
%         poseOut(3) = poseOut(3) + 2*pi; 
%     end

    poseOut(3) = atan2(sinSum, cosSum);

    poseCovOut = zeros(numel(poseIn),numel(poseIn));
    for i = 1:1:size(particles,2)
        err = particles(:,i) - poseOut;
        err(3) = computeAngleDiff(poseOut(3), particles(3,i)); 
    
        poseCovOut = poseCovOut + weights(i).*(err)*(err)';
    end
    
    %% resampling
    M = size(particles,2);
    Neff = 1/sum(weights.^2);
    resample_percentage = 0.70;
    Nt = resample_percentage*M;
    if Neff < Nt
        % resampling
        if true
            Xnew = zeros(size(particles,1),M);
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
                Xnew(:,m) = particles(:,i);
            end
            particles = Xnew;
            weights = ones(M,1)./M;
        end
    end
end
