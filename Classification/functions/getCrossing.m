function crossArray = getCrossing(Th, dist)
    crossing = dist>=Th;
    numCross = 0;
    crossArray = [];
    for i = 1:length(crossing)-1
        if (crossing(i) ~= crossing(i+1)) || (i==1 && crossing(i) == 1)
            numCross = numCross + 1;
            crossArray(numCross) = i;
            %if show xline(x(i), '--'); end
        end 

    end

    if mod(length(crossArray),2) == 1 %Uneven length, the endpoint of the last 
        crossArray(end+1) = crossArray(2) + length(dist);
        crossArray = crossArray(3:end);
    end
    
    %Fix 
    crossArray  = reshape(crossArray, 2, length(crossArray)/2);
    crossArray = transpose(crossArray);
    
    x = linspace(0,4*pi, 2*length(dist));
    crossArray = -x(crossArray(:,:));
    crossArray = crossArray - 0.5*pi;
    %crossArray(end) = crossArray(end) - 2*pi;
       
       
end