clear
clc
close all


load ExtractedWidths\Correct_Final.mat
maxWidthsCorrect = maxWidths;
load ExtractedWidths\Faulty_Final.mat
maxWidthsFaulty = maxWidths;

%Show distribution
figure()
plot(maxWidthsCorrect)
figure()
plot(maxWidthsFaulty)


%Clip values of maxWidhtsFaulty
faultyWidths = min(maxWidthsFaulty, 60);


figure()
hold on

histogram(maxWidthsCorrect,length(unique(maxWidthsCorrect)),'FaceAlpha',0.5, 'Normalization', 'probability')
histogram(faultyWidths,length(unique(maxWidthsFaulty)),'FaceAlpha',0.5, 'Normalization', 'probability')

legend('Correct', 'Faulty')
ylabel('Probability')
xlabel('Maximum width of the bead, saturated at 60 (pixels)')






[Th, I] = min(maxWidthsFaulty);



FP = sum(maxWidthsCorrect>=Th);
TP = sum(maxWidthsFaulty>=Th);
TN = sum(maxWidthsCorrect<Th);
FN = sum(maxWidthsFaulty<Th);

accuracy = (TP + TN) / (FP + TP + TN + FN);

%





