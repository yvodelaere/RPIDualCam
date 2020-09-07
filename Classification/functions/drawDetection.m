function drawDetection(img, cx,cy,rin, crossArray, width, color, lw)
    figure
    imshow(img)
    hold on
    nsegments = 100;
    if size(crossArray,1) >=2
        titleString = sprintf("Detected %d failures", size(crossArray,1));
    elseif size(crossArray,1) == 1
        titleString = sprintf("Detected %d failure", size(crossArray,1));
    else
        titleString = "No failures detected";
    end
    %title(titleString)
    for i = 1:size(crossArray,1)
        r = rin - width;
        ca = crossArray(i,:);
        thetaStart = ca(1);
        thetaEnd = ca(2);
        thetaRange = thetaEnd - thetaStart;
        
        if abs(thetaRange) > 0.1
            thetaStart = thetaStart + 0.1;
            thetaEnd = thetaEnd - 0.1;
            th = thetaStart:thetaRange/nsegments:thetaEnd;
            xunit = r * cos(th) + cx;
            yunit = r * sin(th) + cy;


            coordBegin_inner = [xunit(1), yunit(1)];
            coordEnd_inner = [xunit(end), yunit(end)];
            plot(xunit, yunit, 'Color', color, 'LineWidth', lw);

            r = r + 2*width;
            thetaRange = thetaEnd - thetaStart;
            th = thetaStart:thetaRange/nsegments:thetaEnd;
            xunit = r * cos(th) + cx;
            yunit = r * sin(th) + cy;
            plot(xunit, yunit, 'Color', color, 'LineWidth', lw);

            coordBegin_outer = [xunit(1), yunit(1)];
            coordEnd_outer = [xunit(end), yunit(end)];

            plot([coordBegin_inner(1), coordBegin_outer(1)], [coordBegin_inner(2), coordBegin_outer(2)], 'Color', color, 'LineWidth', lw);
            plot([coordEnd_inner(1), coordEnd_outer(1)], [coordEnd_inner(2), coordEnd_outer(2)], 'Color', color, 'LineWidth', lw);
    
        end
    end
end