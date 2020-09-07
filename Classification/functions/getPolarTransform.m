function [polarImg, outRadi, cx, cy] = extractROI(img,scanScale, outScale, radiMargin)
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
    outRadi = outScale * radii(1)/scanScale;
    scaleCenter =  outScale*centers(1,:)/scanScale;
    newImg = imresize(img,outScale);
    
    

    %% Polar transform

    polarImgBase = newImg;
    cx=scaleCenter(1);
    cy=scaleCenter(2);
    diagImg = norm(size(newImg));
    margin = radiMargin*diagImg;
    in = outRadi - margin;
    out = outRadi + margin;
    phi0 = 1/2*pi;
    phiEnd=  phi0+2*pi;

    polarImg = transImageInvPolar(double(polarImgBase), cx, cy, in, out, phi0, phiEnd, 0);
end