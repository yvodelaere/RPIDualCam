function mask = getMask(imsize, yStart, yStop, centerUp, centerDown, centerUp2, centerDown2, r, r2)
    mask = zeros(imsize);
    mask(yStart:yStop,:) = 1;
    mask = uint8(mask);
    maskUp = createCirclesMask(imsize,centerUp,r);
    maskDown = createCirclesMask(imsize,centerDown,r);
    maskUp2 = createCirclesMask(imsize,centerUp2,r2);
    maskDown2 = createCirclesMask(imsize,centerDown2,r2);
    mask = mask + uint8(maskUp) + uint8(maskDown) + uint8(maskUp2) + uint8(maskDown2);
    mask = uint8(~mask);
end


