function [idEst, cornersEst] = imgRecognition(im,cameraMatrix,distorsionCoeff)
%imgRecognition:
%   1. do threshold; done
%   2. do component-connected extraction; done
%   3. do approximate by Douglas?Peucker algorithm to get convex; done
%   4. do subpixel corner detection
%   5. OCR recognition 
    
    %% here, I just use a global threshold, since the case is very perfect
    threshold = 100;
    imbinary = uint8(im);
    id = imbinary > threshold;
    imbinary(id) = 255;
    imbinary(~id) = 0;
    
%     figure
%     imshow(imbinary,[]);
    
    %% do compnent-connected 
    cstack = zeros(size(imbinary,1)*size(imbinary,2),2);
    cstackID = 0;
    
    cid = uint32(zeros(size(imbinary,1),size(imbinary,2)));
    rtype = int32(zeros(size(imbinary,1)*size(imbinary,2),1));
    componentList = [];%% extracted component-connected
    minRegionSize = 10;
    height = size(imbinary,1);
    width = size(imbinary,2);
    label = 1;
    for i = 1:1:height
        for j = 1:1:width
            if imbinary(i,j) ~= 255 || cid(i,j) ~= 0
%                 if rtype(cid(i,j)) == -1
%                     imbinary(i,j) = 0;%% small region, delete
%                 end
                continue;
            end            
            %% a new unlabeled pixel
            regionSize = 0;
            cid(i,j) = label;
            cstackID = cstackID + 1;
            cstack(cstackID,:) = [i j];
            componentPixels = [];
            contourPixels = [];
            %% do a depth-first search
            while cstackID > 0
                regionSize = regionSize + 1;
                curpixel = uint16(cstack(cstackID,:));
                cstackID = cstackID - 1;
%                 stack(end,:) = [];
                cv = curpixel(1);
                cu = curpixel(2);
                componentPixels = [componentPixels;[cv, cu]];
                
                if (cv-1 > 0 && imbinary(cv-1,cu) == 0 ...
                 || cv+1 <= height && imbinary(cv+1,cu) == 0 ...
                 || cu-1 > 0 && imbinary(cv,cu-1) == 0 ...
                 || cu+1 <= width && imbinary(cv, cu+1) == 0)
                    contourPixels = [contourPixels;[cv, cu]];
                end
                
                %% do 4 neighbors check
                if (cv-1 > 0 && imbinary(cv-1,cu) == 255 && cid(cv-1,cu) == 0)
                    cid(cv-1,cu) = label;
                    cstackID = cstackID + 1;
                    cstack(cstackID,:) = [cv-1,cu];
                end
                if (cv+1 <= height && imbinary(cv+1,cu) == 255 && cid(cv+1,cu) == 0)
                    cid(cv+1,cu) = label;
                    cstackID = cstackID + 1;
                    cstack(cstackID,:) = [cv+1,cu];
                end
                if (cu-1 > 0 && imbinary(cv,cu-1) == 255 && cid(cv,cu-1) == 0)
                    cid(cv,cu-1) = label;
                    cstackID = cstackID + 1;
                    cstack(cstackID,:) = [cv,cu-1];
                end
                if (cu+1 <= width && imbinary(cv,cu+1) == 255 && cid(cv,cu+1) == 0)
                    cid(cv,cu+1) = label;
                    cstackID = cstackID + 1;
                    cstack(cstackID,:) = [cv,cu+1];
                end 
            end
            if (regionSize < minRegionSize)
                rtype(label) = -1;
            else
                componentPixels = uint32(componentPixels);
                contourPixels = uint32(contourPixels);
                component = struct('label',label,'size',regionSize,'pixels',componentPixels, ...
                    'contours',contourPixels);
                componentList = [componentList;component];
            end
            label = label + 1;
        end
    end

%     imdebug = uint8(zeros(height, width));
%     imshow(imdebug,[]);
%     hold on;    
%     for i = 1:1:size(componentList,1)
%         hold on;plot(componentList(i).contours(:,2), ...
%                 componentList(i).contours(:,1),'b.');    
%     end

    %% Douglas?Peucker algorithm
    addpath([pwd filesep '3rdparty/'])
    rectangleCorners = [];
    for i = 1:1:size(componentList,1)
        polygon_indices = boundary(double(componentList(i).contours(:,1)), ...
            double(componentList(i).contours(:,2)), 0.3);
        polygon = double(componentList(i).contours(polygon_indices,:));
        polygon = polygon(1:end-1,:);
%         hold on;plot(polygon(:,2),polygon(:,1),'b.');
        [list_indices, pnts] = douglas_peucker(polygon, 1);
        %% merge
        if (sum(pnts(1,:)-pnts(end,:).^2) < 2)
            pnts = pnts(1:end-1,:);
        end
%         pnts = uint32(pnts);
        if size(pnts,1) == 4 %% not rectangle, filter
%             hold on;plot(pnts(:,2),pnts(:,1),'r-');
            bbwidth = max(pnts(:,2)) - min(pnts(:,2));
            bbheight = max(pnts(:,1)) - min(pnts(:,1));
            
            if abs(bbwidth/bbheight - 1) < 0.2
                rectangleCorners = pnts;
            end
        end
    end
    
    if isempty(rectangleCorners)
        cornersEst = [];
        idEst = [];
        return;
    end
    
    %% subpixel refinement
    R = rectangleCorners(:,1);
    C = rectangleCorners(:,2);
    [rs, cs] = subpix2d(R, C, im);
    cornersEst = [rs' cs'];
    
    %% do OCR recognition
    center = [sum(rs)*0.25 sum(cs)*0.25];
    %% estimate bb
    bbwidth = max(pnts(:,2)) - min(pnts(:,2));
    bbheight = max(pnts(:,1)) - min(pnts(:,1));
    idbb = [round(bbheight*0.6), round(bbwidth*0.6)];
    centerInt = floor(center);
    idimg = imbinary(centerInt(1)-idbb(1):centerInt(1)+idbb(1), centerInt(2)-idbb(2):centerInt(2)+idbb(2));
    idimg = imresize(idimg,[20,20]);
%     imshow(idimg,[]);
    idimg = double(idimg(:)');
    global OCRMDL
    [idEst,score, cost] = predict(OCRMDL,idimg);
end

