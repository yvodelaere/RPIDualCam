function [dist, diffDist, BW, onlyUp, gapMask] = extractFeature(BW, windowSize, visualize,doFilter)
    
   
    [pixels(:,2),pixels(:,1)] = find(BW==1);
    pixels = sortrows(pixels,[1,2]);

    uniqueX = unique(pixels(:,1));



    arrayCnt = 1;
    %Loop through pixels

    gapMask = zeros(size(BW), 'logical');
    sumVal = 0;
    for i = 1:length(pixels)
        if i == 1 % store first minimum
            iMin = i;
            outPix(arrayCnt,1) = i;
            outPix(arrayCnt,2) = pixels(i,2);
            arrayCnt = arrayCnt + 1;
        end
        %detect change in X
        if  i>1
            if pixels(i-1,1) ~= pixels(i,1) %edge
                iMax = (i-1);
                yMin = pixels(iMin,2);
                yMax = pixels(iMax,2);

                lst = pixels(iMin:iMax,2);
                for k = yMin:yMax
                    if ~ismember(k,lst)
                        %Missing value
                        sumVal = sumVal + 1; 
                        gapMask(k, pixels(i-1,1))  = 2;
                    end
                end
                %Store max of i-1
                outPix(arrayCnt,1) = pixels(i-1,1);
                outPix(arrayCnt,2) = pixels(i-1,2);
                arrayCnt = arrayCnt +1;
                %Store min of i
                outPix(arrayCnt,1) = pixels(i,1);
                outPix(arrayCnt,2) = pixels(i,2);
                arrayCnt = arrayCnt +1;
                iMin = i;
            end
        end
    end


    %% Step 5:  Calculate width of the area
    checkPts = 2:2:size(outPix);

    for i = 1:length(checkPts)
        checkPt = checkPts(i);
        dist(i) = outPix(checkPt,2) - outPix(checkPt-1,2);
    end
    
 
 

    %% Step 6: Postprocess signal
    padN = windowSize;
    pad_pre = dist(end-padN+1:end);
    pad_after = dist(1:padN);
    dist = [pad_pre,dist, pad_after];

    %Calculate derivative
    diffDist = diff(dist);
    %Apply filter
    b = (1/windowSize)*ones(1,windowSize);
    a = 1;
    
    

    if doFilter
        dist = filter(b,a,dist);
    end
    diffDist = diff(dist);
    %Shrink 
    dist = dist(padN+1:end-padN);
    diffDist = diffDist(padN:end-padN);
    
    
    
    
    %% Make zero mean
    zeroMean = dist - mean(dist);
    onlyUp = max(zeroMean, 0);
    


    %% Step 7: (Optional) Show extracted signals
    if visualize
    x = linspace(0,2*pi, length(dist));
    figure
    subplot(5,1,2)
    imshow(BW)
    title('Binary rectified image')

    %Derivative of distance
    subplot(5,1,3)
    plot(x,dist)
    hold on;
    ylabel('Width [pixels]')
    xticks([0, 1/2*pi, pi, 3/2*pi, 2*pi])
    xlim([0,2*pi]);
    ylim([0,200])
    ax = gca;
    ax.XTickLabel = {'0','\pi/2','\pi','3\pi/2','2\pi'};
    title('Filtered width profile in pixels')

    subplot(5,1,4)
    plot(x,diffDist)
    hold on;
    ylabel('Derivative of the width[pixels]')
    xticks([0, 1/2*pi, pi, 3/2*pi, 2*pi])
    xlim([0,2*pi]);
    ylim([-1,1])
    ax = gca;
    ax.XTickLabel = {'0','\pi/2','\pi','3\pi/2','2\pi'};
    title('Filtered derivative width profile in pixels')
    
    
    subplot(5,1,5)
    plot(x,onlyUp)
    hold on;
    ylabel('Width [pixels]')
    xticks([0, 1/2*pi, pi, 3/2*pi, 2*pi])
    xlim([0,2*pi]);
    ylim([0,200])
    ax = gca;
    ax.XTickLabel = {'0','\pi/2','\pi','3\pi/2','2\pi'};
    title('Zero mean only up')

    
    
    end
end