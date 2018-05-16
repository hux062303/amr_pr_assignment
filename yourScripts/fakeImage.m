function [im, corners, id] = fakeImage(realPose, camRelPos, camRelRot, cameraMatrix, distorsionCoeff, markerPosition)
%% fake an image with know marker position
camPos = [realPose(1:2)' 0] + camRelPos;

Rot = [cos(realPose(3)) sin(realPose(3)) 0;
      -sin(realPose(3)) cos(realPose(3)) 0;
                      0                0 1];
k1=distorsionCoeff(1);
k2=distorsionCoeff(2);
k3=distorsionCoeff(5);
p1=distorsionCoeff(3);
p2=distorsionCoeff(4);

imWidth = cameraMatrix(1,3)*2;
imHeight = cameraMatrix(2,3)*2;

im = zeros(imHeight,imWidth);
corners = [];
id = [];
global hmarkerSize
imwidth = [];
f = cameraMatrix(1,1);
boardMargin = 10;
for i = 1:1:size(markerPosition, 1)
    cornersInIm = zeros(4,2);
    depth = 0;
    for j = 1:1:4
        pt = [markerPosition(i,j,1) markerPosition(i,j,2) markerPosition(i,j,3)];
        ptInCam = camRelRot * Rot * [pt' - camPos'];
        depth = ptInCam(3);
        xh = ptInCam(1)/ptInCam(3);
        yh = ptInCam(2)/ptInCam(3);
        r = xh*xh+yh*yh;
        xhh = xh*(1+k1*r*r+k2*r^4+k3*r^6)+2*p1*xh*yh+p2*(r^2+2*xh^2);
        yhh = yh*(1+k1*r*r+k2*r^4+k3*r^6)+2*p2*xh*yh+p1*(r^2+2*yh^2);

        %% homogeneous
        pixel = cameraMatrix*[xhh;yhh;1];
        u = pixel(1);v=pixel(2);
        cornersInIm(j,:) = [u,v];
    end
    idu = cornersInIm(:,1) > boardMargin & cornersInIm(:,1) < imWidth - boardMargin;
    idv = cornersInIm(:,2) > boardMargin & cornersInIm(:,1) < imHeight - boardMargin;
    if length(find(idu))~=4 || length(find(idv))~=4
        continue;
    else
        if (round(hmarkerSize*2/depth*f) < 30)
            continue;
        end
        imwidth = [imwidth;round(hmarkerSize*2/depth*f)];
        corners = [corners;cornersInIm];
        id = [id;i];
    end
end

corners = round(corners);
global idBasePath
margin = 5;
%% estimate homography
for i = 1:1:size(id,1)
    idimg = imread(strcat(pwd,idBasePath,int2str(id),'.jpg'));
    idimg = imresize(idimg,[20,20]);
    %% make binary
    idimg(idimg>80) = 255;
    idimg(idimg<=80) = 0;
    fakedMarkerImage = uint8(zeros(imwidth(i),imwidth(i))*255);
    %% make border white
    fakedMarkerImage(1:margin,:) = 255;
    fakedMarkerImage(imwidth-margin:imwidth,:) = 255;
    fakedMarkerImage(:,1:margin) = 255;
    fakedMarkerImage(:,imwidth-margin:imwidth) = 255;
    %% warp id inside
    centeru = floor(imwidth/2);
    centerv = floor(imwidth/2);
    fakedMarkerImage((centerv-9):centerv+10,(centeru-9):centeru+10) = idimg;
    %% 3d
    obj3d(:,1) = [1 1 1]';
    obj3d(:,2) = [1 imwidth(i) 1]';
    obj3d(:,3) = [imwidth(i) imwidth(i) 1]';
    obj3d(:,4) = [imwidth(i) 1 1]';
    %% 2d
    img2d(:,1) = [corners((i-1)*4+1,2) corners((i-1)*4+1,1) 1]';
    img2d(:,2) = [corners((i-1)*4+2,2) corners((i-1)*4+2,1) 1]';
    img2d(:,3) = [corners((i-1)*4+3,2) corners((i-1)*4+3,1) 1]';
    img2d(:,4) = [corners((i-1)*4+4,2) corners((i-1)*4+4,1) 1]';
    
    %%
    H = Hest(obj3d, img2d);
    %%
    T = maketform('projective',H');
    [ImH,XData,YData] = imtransform(fakedMarkerImage,T);
    u = round(XData(1));
    v = round(YData(1));
    im(v+(1:size(ImH,1)),u+(1:size(ImH,2))) = ImH;
end

end
