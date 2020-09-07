clear
clc
close all

addpath('functions')

%% Parameters
show=false;
scanScale = 0.1;
radiMargin = 0.1; %10 percent of diagonal of the image;
outScale = 1;
visualize = false;       %Visualize toggle used for featureExtraction

T = 0.25;
minArea = 100;


%% Load background image
background = imread("D:\20200731_6mm_light\background\background_mean.png");
%Mask image
yStart = 1050;
yStop = 1170;
centerUp = [1830, 525];
centerDown = [1825, 1694];
r=30;
imsize = [size(background,1) size(background,2)];

centerUp2 = [1755 525];
centerDown2 = [1751, 1696];
r2 = 30;
mask = getMask(imsize, yStart, yStop, centerUp, centerDown, centerUp2, centerDown2, r, r2);




%% Load images
folderPath = "D:\20200731_6mm_light";
label = 'faulty';
imgNames = dir(fullfile(folderPath, label, '1*.png'));

maxWidths = [];
for i = 1:length(imgNames)
    i/length(imgNames)
    id = imgNames(i).name;
    imgFile = fullfile(imgNames(i).folder, id);
    img = rgb2gray(imread(imgFile));
    diff = abs(background-img);


    diff = imbinarize(diff,T);
    diff = bwareafilt(diff,[minArea, inf]);
    


    %Extract the width
    segmentedImg = diff.*logical(mask);
    [polarImg, outRadi, cx, cy] = extractROI(segmentedImg,scanScale, outScale, radiMargin);
    [dist, diffDist, BW, onlyUp, gapMask] = extractFeature(polarImg, visualize);
    
    %Store max width
    maxWidths(end+1) =  max(dist);


end

maxWidthsFaulty = maxWidths;
save FaultyMax maxWidths
%%

folderPath = "D:\20200731_6mm_light";
label = 'correct';
imgNames = dir(fullfile(folderPath, label, '1*.png'));

maxWidths = [];
for i = 1:length(imgNames)

  
    i/length(imgNames)
    id = imgNames(i).name;
    imgFile = fullfile(imgNames(i).folder, id);
    img = rgb2gray(imread(imgFile));



    diff = abs(background-img);


    diff = imbinarize(diff,T);

    diff = bwareafilt(diff,[minArea, inf]);

    %Extract the width
    segmentedImg = diff.*logical(mask);
    
    
    
    [polarImg, outRadi, cx, cy] = extractROI(segmentedImg,scanScale, outScale, radiMargin);
    [dist, diffDist, BW, onlyUp, gapMask] = extractFeature(polarImg, visualize);
    
    %Store max width
    maxWidths(end+1) =  max(dist);

end

maxWidthsCorrect = maxWidths;
save CorrectMax maxWidths







