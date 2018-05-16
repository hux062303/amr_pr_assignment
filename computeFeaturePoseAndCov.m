function [featurePoses, featureCovs] = computeFeaturePoseAndCov(Y, weights, SLAM_FEATURE_ID)

    featurePoses = zeros(3,numel(SLAM_FEATURE_ID));
    featureCovs = zeros(3,3,numel(SLAM_FEATURE_ID));
    
    useMax = 0;
    for i = 1:numel(SLAM_FEATURE_ID)
        if useMax == 0
            avgCov = zeros(3,3);
            cosSum = 0;
            sinSum = 0;
            for j = 1:size(Y,1)
                a = [Y{j}.means{i,:}]';
                featurePoses(:,i) = featurePoses(:,i) + weights(j).*a;
                cosSum = weights(j).*cos(a(3));
                sinSum = weights(j).*sin(a(3));
            end
            featurePoses(3,i) = atan2(sinSum, cosSum);
            for j = 1:size(Y,1)
                a = [Y{j}.means{i,:}]';
                err = a - featurePoses(:,i);
                err(3) = computeAngleDiff(a(3), featurePoses(3,i)); 
                avgCov = avgCov + weights(j).*(err)*(err)';
            end
            featureCovs(:,:,i) = avgCov;
        else
            [mw,mid] = max(weights);
            a = [Y{mid}.means{i,:}]';
            featurePoses(:,i) = a;
            featureCovs(:,:,i) = [Y{mid}.covs{i,:}];
        end
    end
end
