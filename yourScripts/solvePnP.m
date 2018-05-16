function [rt_vision,reproj_error] = solvePnP(corners, id, cameraMatrix, distorsionCoeff)
%UNTITLED7 ????????????????????????
%   ????????????????

addpath([pwd filesep '3rdparty/rpnp/func/'])

%% x y yaw, reproj error
rt_vision = zeros(size(id,1),6);
reproj_error = zeros(size(id,1),1);

global hmarkerSize

%% solve planar PnP to get relative r,t from camera to marker
objpts = [-hmarkerSize hmarkerSize 0;hmarkerSize hmarkerSize 0;hmarkerSize -hmarkerSize 0;-hmarkerSize -hmarkerSize 0];

fx = cameraMatrix(1,1);
fy = cameraMatrix(2,2);
cx = cameraMatrix(1,3);
cy = cameraMatrix(2,3);
ifx = 1/fx;
ify = 1/fy;
k1=distorsionCoeff(1);
k2=distorsionCoeff(2);
k3=distorsionCoeff(5);
p1=distorsionCoeff(3);
p2=distorsionCoeff(4);
for i=1:1:size(id,1)
    idp = (i-1)*4+1;
    impts = [corners(idp,1) corners(idp,2);
             corners(idp+1,1) corners(idp+1,2);
             corners(idp+2,1) corners(idp+2,2);
             corners(idp+3,1) corners(idp+3,2);];
    %% undistorsion and normalized
    for j = 1:1:size(impts,1)   
        x = (impts(j,1)-cx)*ifx;
        y = (impts(j,2)-cy)*ify;
        r2 = x*x+y*y;
        icdist = 1/(1+((k3*r2+k2)*r2+k1)*r2);
        deltax = 2*p1*x*y+p2*(r2+2*x*x);
        deltay = p1*(r2+2*y*y)+2*p2*x*y;
        x = (x-deltax)*icdist;
        y = (y-deltay)*icdist;
        impts(j,:) = [x y];
    end
    
    %% solve pnp
    [R, t] = RPnP(objpts', impts');

    so3 = so3_log(R)
    rt_vision(i,:) = [so3' t'];
    
    %% compute reprojection error
    projpts = R' * (objpts') + repmat(-t,1,4);
    projpts = projpts ./ projpts(3,:);
    projpts = projpts';
    error = sum((projpts(:,1)-impts(:,1)).^2)+sum((projpts(:,2)-impts(:,2)).^2);
    reproj_error(i) = error;
end

end

