function estPose = getPoseFromIm(id, se3_vision, camRelPos, camRelRot, markerPosition)

%% x y yaw, reproj error
estPose = zeros(size(id,1),4);

for i=1:1:size(id,1)
    R = so3_exp(se3_vision(i,1:3)');
    t = se3_vision(i,4:6)';
    %% transform to 2d pose
    global R33_Markers realPose
    Rg2m = [R33_Markers(id(i),1,1) R33_Markers(id(i),1,2),R33_Markers(id(i),1,3); ...
            R33_Markers(id(i),2,1),R33_Markers(id(i),2,2),R33_Markers(id(i),2,3); ...
            R33_Markers(id(i),3,1),R33_Markers(id(i),3,2),R33_Markers(id(i),3,3)];
    Rg2b = camRelRot' * R' * Rg2m;
    [yaw, pitch, roll] = dcm2angle(Rg2b);
    
%     if yaw>=0 && yaw <=pi
        estPose(i,3) = yaw;
%     else
%         estPose(i,3) = yaw;
%     end
    
    %% marker center
    maekercenter = zeros(3,1);
    markercenter(1,1) = markerPosition(id(i),1,1)+markerPosition(id(i),2,1)+markerPosition(id(i),3,1)+markerPosition(id(i),4,1);
    markercenter(2,1) = markerPosition(id(i),1,2)+markerPosition(id(i),2,2)+markerPosition(id(i),3,2)+markerPosition(id(i),4,2);
    markercenter(3,1) = markerPosition(id(i),1,3)+markerPosition(id(i),2,3)+markerPosition(id(i),3,3)+markerPosition(id(i),4,3);
    markercenter = markercenter * 0.25;

    camPosG = Rg2m'*R'*(-t + camRelPos') + markercenter;
%     realPose

    estPose(i,1:2) = camPosG(1:2);
end

end
