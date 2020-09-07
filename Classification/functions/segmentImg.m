function segmentedImg = segmentImg(imname, mode, T)
    validStrings = ["manual"];
    validStr = validatestring(mode,validStrings);
    if validStr == "manual"
        disp("Manual segmentation");
        if strcmp(imname,'images/correct.jpg')
            segmentedImg = ~imbinarize(rgb2gray(imread('images/segmented/correct_segmented.png')), T);
        elseif strcmp(imname,'images/f1.jpg')
            segmentedImg = ~imbinarize(rgb2gray(imread('images/segmented/faulty_1.png')), T);
        elseif strcmp(imname,'images/f2.jpg')
            segmentedImg = ~imbinarize(rgb2gray(imread('images/segmented/faulty_2.png')), T);
        end
    end
    
    %Apply morphology on image
    SE = strel('diamond',3);
    segmentedImg = imerode(segmentedImg,SE);
end

