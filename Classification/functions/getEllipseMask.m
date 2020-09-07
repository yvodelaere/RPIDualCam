function [ellipseMask, innerEllipse] = getEllipseMask(baseImage, N, visualize)
    edgeImg = edge(baseImage, 'Sobel');
    [Y,X] = find(baseImage==1);
    whitePoints = [X,Y];
    baseEllipse = fit_ellipse(X,Y);
    middlePoint = [baseEllipse.X0_in, baseEllipse.Y0_in];

    angleQuerys = linspace(0,2*pi,N)';
    minPixels = zeros(N, 3);
    maxPixels = zeros(N, 3);
    for i = 1:length(whitePoints)
            %Calculate angle between center point and white Point
            point = whitePoints(i,:);
            dx = point(1) - middlePoint(1);
            dy = point(2) - middlePoint(2);
            angle = atan2(dy, dx);
            if angle<0
                angle = angle + 2*pi;
            end
            %Calculate distance
            distance = norm(dx,dy);
            [B,I] = min(abs(angle - angleQuerys));
            %Index of angle is I
            if distance < minPixels(I,1) | minPixels(I,1) == 0
                minPixels(I,1) = distance;
                minPixels(I,2:end)= whitePoints(i,:);
            end

            if distance > maxPixels(I,1)
                maxPixels(I,1) = distance;
                maxPixels(I,2:end)= whitePoints(i,:);
            end    
    end

    innerCoords = [minPixels(:,2), minPixels(:,3)];
    outerCoords = [maxPixels(:,2), maxPixels(:,3)];



    % % Remove entries which have zero
    minPixels = minPixels(minPixels(:,1)>0,:);
    maxPixels = maxPixels(maxPixels(:,1)>0,:);

    %Show minPixels
    inner = zeros(size(edgeImg));
    I = sub2ind(size(inner), minPixels(:,2), minPixels(:,3));
    inner(I) = 1;
    %Show minPixels
    outer = zeros(size(edgeImg));
    I = sub2ind(size(inner), maxPixels(:,2), maxPixels(:,3));
    outer(I) = 1;


    %% Fit ellipse using least squares
    innerEllipse = fit_ellipse(minPixels(:,2),minPixels(:,3));
    outerEllipse = fit_ellipse(maxPixels(:,2), maxPixels(:,3));



    if visualize
        figure();
        imshow (baseImage)
        hold on
    end
    ellipseCoords_inner = drawEllipse(innerEllipse,N, visualize);
    ellipseCoords_outer = drawEllipse(outerEllipse,N, visualize);

    innerMask = poly2mask(ellipseCoords_inner(1,:),ellipseCoords_inner(2,:),size(baseImage,1),size(baseImage,2));
    outerMask = poly2mask(ellipseCoords_outer(1,:),ellipseCoords_outer(2,:),size(baseImage,1),size(baseImage,2));
    ellipseMask = outerMask - innerMask;
end
