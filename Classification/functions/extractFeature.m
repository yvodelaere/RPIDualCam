function [dist, diffDist, BW, onlyUp, gapMask] = extractFeature(BW, visualize)
    polarImg = BW;
    widthProfile = zeros(size(polarImg,2),1);
    gapMask = zeros(size(polarImg), 'logical');
    for m = 1:size(polarImg,2)
        [~, top] = max(polarImg(:,m));
        [~, bottom] = max(flip(polarImg(:,m)));
        bottom = size(polarImg,1) - bottom;
        width = bottom - top;
        if top == 0 || bottom == size(polarImg,1)-1 || width  == 1 || width < 16
            widthProfile(m) = 0;
        else
            widthProfile(m) = bottom - top;

            values = polarImg(top:bottom,m);

            idxGap = find(values==0);
            idxGap = idxGap + top -1;
            gapMask(idxGap,m) = 1;  
        end
    end
    %Mask out the derivative
    diffMask = ones(1,length(widthProfile));
    diffMask(widthProfile==0) = 0;
    
    diffMaskL = [diffMask(3:end),1,1];
    diffMaskR = [1,1,diffMask(1:end-2)];
    diffMask = diffMask & diffMaskL;
    diffMask = diffMask & diffMaskR;
    
    
    
    %Put average value where the width is 0
    %First, calculate mean over values which are non zero
    meanValue = sum(widthProfile(widthProfile>0))/length( widthProfile(widthProfile>0));
    %widthProfile(widthProfile==0) = meanValue;
    
 
    dist = widthProfile.';
    diffDist = diff(dist);
    diffDist = diffDist .* diffMask(1:end-1);
    diffDist = abs(diffDist);

    
    
    
    %% Make zero mean
    zeroMean = dist - mean(dist);
    onlyUp = max(zeroMean, 0);
    


    %% Step 7: (Optional) Show extracted signals
    if visualize
    x = linspace(0,2*pi, length(dist));
    figure
    subplot(5,1,2)
    imshow(BW)
    axis normal
    title('Binary rectified image')

    %Derivative of distance
    subplot(5,1,3)
    plot(x,dist)
    hold on;
    ylabel('Width [pixels]')
    xticks([0, 1/2*pi, pi, 3/2*pi, 2*pi])
    xlim([0,2*pi]);
    ylim([0,100])
    ax = gca;
    ax.XTickLabel = {'0','\pi/2','\pi','3\pi/2','2\pi'};
    title('Extracted width profile in pixels')
% 
%     subplot(5,1,4)
%     plot(x(1:end-1),diffDist)
%     hold on;
%     ylabel('Absolute Derivative of the width[pixels]')
%     xticks([0, 1/2*pi, pi, 3/2*pi, 2*pi])
%     xlim([0,2*pi]);
%     ylim([0,15])
%     ax = gca;
%     ax.XTickLabel = {'0','\pi/2','\pi','3\pi/2','2\pi'};
%     title('Absolute Derivative of the width[pixels]')
%     
%     subplot(5,1,5)
%     plot(x,onlyUp)
%     hold on;
%     ylabel('Width [pixels]')
%     xticks([0, 1/2*pi, pi, 3/2*pi, 2*pi])
%     xlim([0,2*pi]);
%     ylim([0,50])
%     ax = gca;
%     ax.XTickLabel = {'0','\pi/2','\pi','3\pi/2','2\pi'};
%     title('Zero mean only up')
%     end
end