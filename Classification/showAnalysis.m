clear
clc
close all

addpath('functions')


load ExtractedWidths\Correct_Final.mat
label = 'correct';
figure()
plot(maxWidths)
%% Parameters
show=false;
scanScale = 0.1;
radiMargin = 0.1; %10 percent of diagonal of the image;
outScale = 1;
visualize = false;       %Visualize toggle used for featureExtraction

blurcof = 3;
%% Load background image
background = imread("D:\20200731_6mm_light\background\background_mean.png");

%background = imgaussfilt(background,blurcof);

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




figure()
imshow(background.*mask)
%% Load images
%Imnumimg
%64 faulty
%129 correct
i = 7;


folderPath = "D:\20200731_6mm_light";

imgNames = dir(fullfile(folderPath, label, '1*.png'));

maxWidths = [];
id = imgNames(i).name;
id
imgFile = fullfile(imgNames(i).folder, id);
img = rgb2gray(imread(imgFile));
%img = imgaussfilt(img,blurcof);


figure()
imshow(img.*mask)

figure()
imshow(img)
title("Original image")



diff = abs(background-img);

figure()
imagesc(diff)
colorbar


figure()
imshow(diff)
title("Difference image")


diff = imbinarize(diff,0.25);
diff = bwareafilt(diff,[100, inf]);







%Extract the width
segmentedImg = diff.*logical(mask);



figure()
B = labeloverlay(img,segmentedImg);
imshow(B)


figure()
imshow(segmentedImg)
title("Difference image after after binarization")

[polarImg, outRadi, cx, cy] = extractROI(segmentedImg,scanScale, outScale, radiMargin);
[dist, diffDist, BW, onlyUp, gapMask] = extractFeature(polarImg, true);


% Step 4: Classification
crossArray = getCrossing(39, dist);


%% Step 5: Show result

drawDetection(img, cx,cy, outRadi, crossArray, 100, 'y', 5)






    






