clear; close all; clc;

load('dateLabels.mat');
load('dateDetected.mat');

%sort dates detected?

detections = dateDetected.FoundDates;
known = dateLabels.Dates;

total = size(detections, 1);
matches = false(total, 1);

for i = 1:total    
    if (size(detections{i}, 1) == 1)
        matches(i) = strcmpi(strrep(detections{i}, ' ', ''), ...
            strrep(known{i}, ' ', ''));
    else
        multiMatches = zeros(1);
        for j = 1:size(detections{i}, 1)
            multiMatches(j) = strcmpi(strrep(detections{i}(j), ' ', ''), ...
                strrep(known{i}, ' ', ''));
        end
        matches(i) = max(multiMatches);
    end
end

accuracy = sum(matches) / total;

fprintf("Text Recognition Accuracy from 500 images = %.4f \n", accuracy);