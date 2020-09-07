function [polarImg, outRadi, cx, cy] = extractROI(img,scanScale, outScale, radiMargin, cx, cy, outRadi)

    %% Extract rough radi
    initImg = imresize(img,scanScale);
    edgeImg = edge(initImg,'Sobel');
    mid = round(size(img,2)/2);
    minrad = round(0.15*size(initImg,2));
    maxrad = round(0.45*size(initImg,2));
    [centers,radii,~] = imfindcircles(edgeImg,[minrad,maxrad], ...
                                           'ObjectPolarity', 'bright', ...
                                           'Sensitivity', 1, ...
                                           'Method', 'PhaseCode', ...
                                           'EdgeThreshold', 0.5);
    %% Alternatively, just scale up the found circle
    if ~exist('outRadi', 'var')
        outRadi = outScale * radii(1)/scanScale;
    end
    scaleCenter =  outScale*centers(1,:)/scanScale;
    newImg = imresize(img,outScale);
    polarImgBase = newImg;
    

    %% Polar transform

        
    if ~exist('cx','var')
        cx=scaleCenter(1);
    end
    if ~exist('cy','var')
        cy=scaleCenter(2);
    end
    


    diagImg = norm(size(newImg));
    margin = radiMargin*diagImg;
    
    
    
    in = outRadi - margin;
    out = outRadi + margin;
    phi0 = 1/2*pi;
    phiEnd=  phi0+2*pi;

    
    polarImg = transImageInvPolar(double(polarImgBase), cx, cy, in, out, phi0, phiEnd, 0);
    polarImg = imbinarize(polarImg,0);
end