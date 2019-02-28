%% MATLAB Script for Testing the Expiry Date Recognition Algorithm
%Author: Daniel Turner
%University: University of Lincoln
%Date: 25/2/2019

%Functions used to display images are removed since they are irrelevant in
%the actual implementation

%Will produce 8 values:
%PRECISION = Measure of how relevant the results were
%RECALL = Measure of how many relevant were returned
%RECOGNITION ACCURACY = How many dates were correct out of the correctly
%identified bounding boxes
%OVERALL ACCURACY = How many dates were correct out of the entire dataset
%AVERAGE TIME = The mean time for the algorithm to be completed
%MINIMUM TIME = The fastest time the algorithm was completed
%MAXIMUM TIME = The slowet time the algorithm was completed
%STANDARD DEVIATION = The amount of varition in time complexity

%Reset MATLAB environement
clear; close all; clc;

%% Read Ground Truths
%########################################################################
%WARNING: you will need to change the local filepaths for the groundTruth 
%object to the correct directory on your system. Use: changeFilePaths() 
%########################################################################

%Loads a table holding the Ground Truth Bounding Boxes for images in dataset
%Need to change directories to a valid path to dataset
load('dateGroundTruths.mat');
%Loads a table holding the Expiry Date values for images in the dataset
load('dateLabels.mat');

%Get size of dataset
imageCount = size(gTruth.LabelData, 1);

%Initialise metric vectors
precision = zeros(imageCount, 1);
recall = zeros(imageCount, 1);
expiryDates = cell(imageCount, 1);
dateKnown = dateLabels.Dates;
times = zeros(imageCount, 1);

%% Execute Algorithm
%For loop is parallelised to decrease computation time for large dataset.
%Parallel Computing Toolbox is required. Alternatives: 'for' 

%Iterate through the size of the dataset
parfor iteration = 1:imageCount
    %% Measure Execution Time (1/2)
    %Start the stopwatch
    tic;
    
    %% Read Image
    %Load image from filepath resent in ground truth object
    I = imread(gTruth.DataSource.Source{iteration});
    
    %% Convert to greyscale
    %Check if image is RGB denoted by being 3D array
    if size(I,3) > 0
        grey = rgb2gray(I);
    else
        grey = I;
    end
    
    %Get dimensions of image
    [imageHeight, imageWidth] = size(grey);
    
    %% Image Pre-processing - Remove Noise, Increase Contrast & Sharpness
    
    %Perform Linear Spatial Filtering to eliminate noise
    %Weiner removes gaussian & speckle noise while preserving edges by adapting
    %smoothing amount
    greyWeiner = wiener2(grey, [3 3]);
    
    %Perform Linear Contrast Stretching (Could change Gamma?)
    greyContrastStretch = imadjust(greyWeiner);
    
    %Use unsharp masking to increase image sharpness
    greySharp = imsharpen(greyContrastStretch);
    
    %% Maximally Stable Extremal Regions (MSER)
    
    %Detect MSER Regions
    %RegionAreaRange = region min|max size (30 14000)
    %ThresholdDelta = Step size between intensity threshold (2)
    %MaxAreaVariation = max area variation between regions (0.25)
    mserRegions = detectMSERFeatures(greySharp, 'RegionAreaRange', ...
        [150 1500], 'ThresholdDelta', 2.5, 'MaxAreaVariation', 0.2);
    
    %Concatenate pixel coordinates as Nx2 matrix
    mserPixels = vertcat(cell2mat(mserRegions.PixelList));
    
    %Initialise logical image with necessary dimensions
    mserBW = false(imageHeight, imageWidth);
    %Convert img co-ordinates to linear image indexes
    ind = sub2ind(size(mserBW), mserPixels(:,2), mserPixels(:,1));
    %assign true to co-ordinates that match
    mserBW(ind) = true;
    
    %% Image Post-Processing - Opening,
    
    %Define 3x3 square structuring element
    seSquare = strel('square', 3);
    
    %Opening to remove small joins
    opened = imopen(mserBW, seSquare);
    
    %Remove small blobs below 100 pixels
    clearNoise = bwareaopen(opened, 100);
    %Close small holes by inverting image between foreground and background
    clearSmallHoles = ~bwareaopen(~clearNoise, 3);
    
    %% Connected Component Enhanced MSER (CCEMSER)
    
    %Get the image containg solely the object and its bounding box
    mserStats = regionprops(clearSmallHoles, 'Image', 'BoundingBox');
    %Initialise variables
    totalObjects = size(mserStats, 1);
    CCadjustedImage = false(imageHeight, imageWidth);
    
    for i = 1:totalObjects
        %Get BBox and image
        %Correct for slightly large BBox and round floating values
        imageBBox = ceil(mserStats(i).BoundingBox - [0, 0, 1, 1]);
        image = mserStats(i).Image;
        
        %Crop the greyscale image
        greyImage = imcrop(greySharp, imageBBox);
        
        %Threshold the cropped greyscale image using Otsu's Method
        binaryImage = imbinarize(greyImage);
        
        %Perform CC analysis to find most likely text (BoW, DoL text)
        ccReg = bwconncomp(image & binaryImage);
        ccInv = bwconncomp(image & ~binaryImage);
        
        %Concatenate list to handle multiple objects in image
        ccRegSize = size(cat(1, ccReg.PixelIdxList{:}), 1);
        ccInvSize = size(cat(1, ccInv.PixelIdxList{:}), 1);
        
        %Check objects to find which has the fewest non-zero components which
        %will more than likely have the appropriate threshold
        if ccReg.NumObjects == 0 && ccInv.NumObjects ~= 0
            %Use inverse image
            keepImage = image & ~binaryImage;
        elseif ccInv.NumObjects == 0 && ccReg.NumObjects ~= 0
            %Use regular image
            keepImage = image & binaryImage;
        elseif ccReg.NumObjects < ccInv.NumObjects
            %Use regular image
            keepImage = image & binaryImage;
        elseif ccInv.NumObjects < ccReg.NumObjects
            %Use inverse image
            keepImage = image & ~binaryImage;
            %Edge case where they could have matching number of objects
        elseif ccInv.NumObjects == ccReg.NumObjects
            %Choose threshold that procuces the most pixels
            if ccRegSize < ccInvSize
                keepImage = image & ~binaryImage;
            else
                keepImage = image & binaryImage;
            end
        else
            %Use regular image as last resort
            keepImage = image & binaryImage;
        end
        
        %Replace existing image locations with new enhanced images
        CCadjustedImage(imageBBox(2):imageBBox(2) + imageBBox(4), ...
            imageBBox(1):imageBBox(1) + imageBBox(3)) = keepImage;
    end
    
    %Remove small blobs
    clearNoise = bwareaopen(CCadjustedImage, 20);
    %Close small holes by inverting image between foreground and background
    clearSmallHoles = ~bwareaopen(~clearNoise, 3);
    
    %% Remove Unlikely Candidates using Region Properties
    
    % Eccentricity = similarity to line segment (1) or cirlce (0)
    % Euler Number = Don't have many holes (max 2 | B)
    % Aspect Ratio = mostly square (vertical & horizontal)
    % Extent = have very high or very low occupation of bounding box (O vs l)
    
    %Label the image and get the properties of each object
    mserLabel = bwlabel(clearSmallHoles);
    mserStats = regionprops(clearSmallHoles, 'BoundingBox', 'Eccentricity', ...
        'EulerNumber', 'Extent', 'Solidity');
    
    %Calculte the maximum horizontal/vertical aspect ratio
    bBoxes = vertcat(mserStats.BoundingBox);
    bbWidths = bBoxes(:, 3)';
    bbHeights = bBoxes(:, 4)';
    aspectRatio = max(bbWidths ./ bbHeights, bbHeights ./ bbWidths);
    
    %Remove blobs that have more than 3 holes in them. Need to compensate for
    %noise
    validEulerNo = [mserStats.EulerNumber] >= -2;
    %Remove blobs that are perfect lines (eg. barcodes)
    validEccentricity = [mserStats.Eccentricity] < 0.99;
    %Letters should have normal distribution of Area to BBox
    validExtent = [mserStats.Extent] > 0.25 & [mserStats.Extent] < 0.9;
    %The ratio between the region and the convex hull
    validSolidity = [mserStats.Solidity] > 0.4;
    %Make use of vertical and horizontal aspect ratio to ensure shape are
    %roughly square = 1
    validAspectRatio = aspectRatio < 3;
    
    %Find the index of all objects that have valid properties
    keptObjects = find(validEulerNo & validEccentricity & validExtent & ...
        validSolidity & validAspectRatio);
    %Return pixels that satisfy the similar property criteria for the image
    keptObjectsImage = ismember(mserLabel, keptObjects);
    
    %% Stroke Width Transform
    
    %Label the image and get the properties of each object
    swtLabel = bwlabel(keptObjectsImage);
    swtStats = regionprops(keptObjectsImage, 'Image');
    
    %Initialise variables
    totalObjects = size(swtStats, 1);
    swtVariation = zeros(1, totalObjects);
    
    %Define the maximum variation in stroke width for letters
    swtVariationThresh = 0.4;
    
    for i = 1:totalObjects
        %Get the image conting just the object
        object = swtStats(i).Image;
        
        %Pad the image with 0's to avoid stroke width being affected by boundary
        paddedObject = padarray(object, [1 1]);
        
        %Distance Transform
        distanceTransform = bwdist(~paddedObject);
        
        %Perform thinning until Skeleton
        skeletonTransform = bwmorph(paddedObject, 'thin', inf);
        
        %Retrieve the stroke width values for the image
        swtWidths = distanceTransform(skeletonTransform);
        
        %Calculate the variation in stroke widths
        swtVariation(i) = std(swtWidths)/mean(swtWidths);
    end
    
    %Find stroke widths that are below the variation threshold
    validStrokeWidths = swtVariation < swtVariationThresh;
    %Find the index of these objects
    keptSWT = find(validStrokeWidths);
    %Create an image made of objects that are below the variation threshold
    keptSWTImage = ismember(swtLabel, keptSWT);
    
    %% Rule-Based Candidate Text Grouping
    
    %Get the bounding box for each object
    textStats = regionprops(keptSWTImage, 'BoundingBox');
    textROI = vertcat(textStats.BoundingBox);
    
    %Get bounding box sizes
    roiX = textROI(:, 1);
    roiY = textROI(:, 2);
    roiW = textROI(:, 3);
    roiH = textROI(:, 4);
    
    %Expand ROI by 2/3 the character width in horizontal direction since dates
    %are typically vertically aligned
    expandedX = roiX - (roiW * (2/3));
    expandedW = roiW + ((roiW * (2/3)) * 2);
    
    %Ensure that ROI is within bounds of the image
    expandedX = max(expandedX, 1);
    expandedW = min(expandedW, imageWidth - expandedX);
    
    %Create expanded bounding boxes
    expandedTextROI = [expandedX, roiY, expandedW, roiH];

    %Calculate the ratio of union between bounding boxes
    overlapRatio = bboxOverlapRatio(expandedTextROI, expandedTextROI, 'Union');
    overlapSize = size(overlapRatio, 1);
    
    %Remove union with own Bounding Box
    overlapRatio(1:overlapSize + 1:overlapSize^2) = 0;
    
    %Create node graph of connected Bounding Boxes & find the index of boxes
    %that intersect
    labelledROI = conncomp(graph(overlapRatio));
    
    %Ensure that there is similarity between connected letters
    maxComponents = max(labelledROI);
    %loop through connected bounding boxes
    for i = 1:maxComponents
        %find the index of connected bounding boxes
        connectedBoxes = find(labelledROI == i);
        %Get the bounding box heights
        heightOfBoxes = roiH(connectedBoxes);
        
        %Ensure that joined regions have similar heights
        meanHeight = mean(heightOfBoxes);
        meanError = meanHeight/2;
        
        %Create mask of bounding boxes that don't meet the criteria
        validBoxes = heightOfBoxes > meanHeight - meanError & heightOfBoxes < meanHeight + meanError;
        discardHeight = find(validBoxes == 0);
        
        %Get the index of bounding boxes that don't meet the criteria
        invalidHeight = connectedBoxes(discardHeight);
        
        %Check that there the validHeights matrix is not empty
        if (~isempty(invalidHeight))
            %Loop through invalid indexes
            for j = 1:size(invalidHeight, 2)
                id = invalidHeight(j);
                %Seperate the component into a new indices by creating
                %a new 'label' above max value
                labelledROI(id) = max(labelledROI) + 1;
            end
        end
    end
    
    %Find the minimum values of X & Y and the maximum values of W & H for each
    %of the intersecting bounding boxes to form encompassing bounding boxes
    labelledROI = labelledROI';
    x1 = accumarray(labelledROI, expandedX, [], @min);
    y1 = accumarray(labelledROI, roiY, [], @min);
    x2 = accumarray(labelledROI, expandedX + expandedW, [], @max);
    y2 = accumarray(labelledROI, roiY + roiH, [], @max);
    
    %Create merged bounding boxes in [X Y H W] format
    mergedTextROI = [x1, y1, x2 - x1, y2 - y1];
    
    %Calculate size of labels after updating connected bounding boxes
    labelSizes = hist(labelledROI', 1:max(labelledROI));
    
    %Remove single, unconnected bounding boxes
    wordCandidates = labelSizes > 1;
    filteredTextROI = mergedTextROI(wordCandidates, :);
    
    %Get the bounding boxes of removed objects
    removePixelsROI = mergedTextROI(~wordCandidates, :);
    %Ensure the bounding box isn't empty
    removePixelsROI(all(~removePixelsROI, 2), : ) = [];
    
    %Get the [X Y H W] values of bounding box accounting for real values
    removeMinX = ceil(removePixelsROI(:, 1));
    removeMaxX = round(removeMinX + removePixelsROI(:, 3));
    removeMinY = ceil(removePixelsROI(:, 2));
    removeMaxY = round(removeMinY + removePixelsROI(:, 4));
    
    %Remove single, unconnected bounding boxes for binary image
    removeROIImage = keptSWTImage;
    for i = 1:size(removePixelsROI, 1)
        removeROIImage(removeMinY(i):removeMaxY(i), removeMinX(i):removeMaxX(i)) = 0;
    end
    
    %Expand a the bounding box vertically to fully contain the text's height
    pixelExpansion = 2;
    expandedY = filteredTextROI(:, 2) - pixelExpansion;
    expandedH = filteredTextROI(:, 4) + (2 * pixelExpansion);
    %Ensure that ROI is within bounds of the image
    expandedY = max(expandedY, 1);
    expandedH = min(expandedH, imageHeight - expandedH);
    
    %Create expanded bounding boxes in appropriate format
    expandedFilteredTextROI = [filteredTextROI(:, 1), expandedY, ...
        filteredTextROI(:, 3), expandedH];
    
    %% Perform Optical Character Recognition (OCR)
    
    %Get text ROIs to perform OCR on
    ocrROI = [expandedFilteredTextROI(:, 1), expandedFilteredTextROI(:, 2), ...
        expandedFilteredTextROI(:, 3), expandedFilteredTextROI(:, 4)];
    
    %Pre-allocate vectors & initialise variables
    ROISize = size(ocrROI, 1);
    detectedText = strings(ROISize, 1);
    samplePoints = 10;
    
    %Loop through candidate words
    for i = 1:ROISize
        %Crop binary image using expanded bounding boxes
        ROI = imcrop(removeROIImage, [ocrROI(i, 1), ocrROI(i, 2), ocrROI(i, 3), ...
            ocrROI(i, 4)]);
        
        %Remove pixels touching the edge since they are probably not related to
        %text
        ROI = imclearborder(ROI);
        
        %Get minimum Bounding Boxes for each letter
        ROIstats = regionprops(ROI, 'BoundingBox');
        
        %Check that there is more than one object in image
        if (size(ROIstats, 1) <= 1)
            %skip iteration since a date won't be one letter long & can't
            %interpolate a line of best fit from one point
            continue
        end
        
        %Convert to 4xN matrix for operations
        letterBoxes = vertcat(ROIstats.BoundingBox);
        
        %Get the centre of the bounding boxes. This is better than centroids as
        %the centre of gravity is not the true centre.
        centreX = round(mean([letterBoxes(:, 1), letterBoxes(:, 1) + letterBoxes(:, 3)], 2));
        centreY = round(mean([letterBoxes(:, 2), letterBoxes(:, 2) + letterBoxes(:, 4)], 2));
        
        %Calculate line of best fit coeffecient using the centre of letters
        bestFit = polyfit(centreX, centreY, 1);
        
        %Create linearly spaced vector of 10 sample points from 1 to width of
        %the image
        xValues = linspace(1, size(ROI, 2), samplePoints);
        
        %Estimate the values of y at x values using the best fit coeffecient
        %to create a line of best fit
        yValues = polyval(bestFit, xValues);
        
        %Calculate the line of best fit's angle
        angle = atan2d(yValues(samplePoints) - yValues(1), ...
            xValues(samplePoints) - xValues(1));
        
        %Don't correct image if it is within 7.5 degrees of 0
        %OCR is capable of reading letters most accurately at < 10 degree offset
        if (angle > 7.5 || angle < -7.5)
            ROI = imrotate(ROI, angle);
        end
        
        %Perform OCR on the image using MATLAB's Tesseract-OCR 3.02 implementation
        %Specifying 'Block' will ensure that it looks for one or more horizontal text lines
        %Specifying the character set will reduce the chance of confusion with
        %characters that cannot be found in dates
        ocrOutput = ocr(ROI, 'TextLayout', 'Block', 'CharacterSet', ...
            '1234567890ABCDEFGHIJLMNOPRSTUVYabcdefghijlmnoprstuvy/.-:');
        
        %Store the detected text
        detectedText(i) = ocrOutput.Text;
    end
    
    %% Perform Text Matching using Regex
    
    %Formats found in more detail in the script...
    
    %Handles formats 1 and 6
    regexTextDate = '(\d{1,2})([\/\\\-\. ]|)(Jan(?:uary)?|Feb(?:ruary)?|Mar(?:ch)?|Apr(?:il)?|May|Jun(?:e)?|Jul(?:y)?|Aug(?:ust)?|Sep(?:tember)?|Oct(?:ober)?|Nov(?:ember)?|Dec(?:ember)?)([\/\\\-\. ]|)((?:\d{2}){0,2})';
    %Handles formats 2
    regexTextYear = '^(Jan(?:uary)?|Feb(?:ruary)?|Mar(?:ch)?|Apr(?:il)?|May|Jun(?:e)?|Jul(?:y)?|Aug(?:ust)?|Sep(?:tember)?|Oct(?:ober)?|Nov(?:ember)?|Dec(?:ember)?)([\/\\\-\. ]|)(?:\d{2}){1,2}';
    %Handles format 3, 4 and 5
    regexNumeric = '(\d{1,2})([\/\\\-\.])(\d{1,2})(\2)((?:\d{2}){1,2})';
    
    %Remove newline characters from the end of the text
    detectedText = strip(detectedText, newline);
    
    %Perfom case-insensitive regular expression matching for dates, returning
    %a cell array of matching text
    validTextDate = regexpi(detectedText, regexTextDate, 'match');
    validTextYear = regexpi(detectedText, regexTextYear, 'match');
    validNumeric = regexpi(detectedText, regexNumeric, 'match');
    
    %Concatenate matching text into string array
    expiryDates{iteration} = string(vertcat(validTextDate{:}, validTextYear{:}, validNumeric{:}));
    
    %% Measure Execution Time (2/2)
    %Sop the stopwatch and record time
    times(iteration) = toc;
    
    %% Calculate Precision and Recall
    
    %Compare the identified bounding boxes with the ground truth using a
    %threshold of 0.5
    [precision(iteration), recall(iteration)] = bboxPrecisionRecall(expandedFilteredTextROI, ...
        gTruth.LabelData{iteration, 1}{1});
    
end
%End of parallel loop

%% Calculate Detection Accuracy

%Calculate the mean precision and recall values across the dataset
avgPrecision = mean(precision);
avgRecall = mean(recall);

%Output results to command window
fprintf("Expiry Date Detection:\nPrecision: %.4f  Recall: %.4f\n\n", ...
    avgPrecision, avgRecall);

%% Calculate Recognition Accuracy

%Store dates in new variable for easier access
dates = expiryDates;
knownDates = dateLabels.Dates;

%Remove dates that were not detected
filter = (recall > 0);
detectedKnown = knownDates(filter);
detectedDates = dates(filter);

%Find new size of dataset after removing undetected dates
detectedTotal = size(detectedKnown, 1);

%Initialise variable to store whether dates match
matches = false(detectedTotal, 1);

%Loop through images with detected dates
for i = 1:detectedTotal
    %If there was only 1 date detected
    if (size(detectedDates{i}, 1) == 1)
        %Compare detected date and true dates (remove all spaces)
        matches(i) = strcmpi(strrep(detectedDates{i}, ' ', ''), ...
            strrep(detectedKnown{i}, ' ', ''));
    else
        %Initialise variable to store whether any of the detected dates match
        multiMatches = zeros(1);
        %Loop through all detected dates in image
        for j = 1:size(detectedDates{i}, 1)
            %Compare detected date and true dates (remove all spaces)
            multiMatches(j) = strcmpi(strrep(detectedDates{i}(j), ' ', ''), ...
                strrep(detectedKnown{i}, ' ', ''));
        end
        %Return true if any of the dates matches
        matches(i) = max(multiMatches);
    end
end

%Find accuracy bycalculating ratio between matches and total detected dates
recogAccuracy = sum(matches) / detectedTotal;

%Output results to command window
fprintf("Expiry Date Recognition:\nAccuracy from %.0f images: %.4f\n\n", ...
    detectedTotal, recogAccuracy);

%% Calculate Overall Accuracy

%Re-initialise variable to store whether dates match
matches = false(imageCount, 1);

%Loop through all images
for i = 1:imageCount
    %If there was only 1 date detected
    if (size(dates{i}, 1) == 1)
        %Compare detected date and true dates (remove all spaces)
        matches(i) = strcmpi(strrep(dates{i}, ' ', ''), ...
            strrep(knownDates{i}, ' ', ''));
    else
        %Initialise variable to store whether any of the detected dates match
        multiMatches = zeros(1);
        %Loop through all detected dates in image
        for j = 1:size(dates{i}, 1)
            %Compare detected date and true dates (remove all spaces)
            multiMatches(j) = strcmpi(strrep(dates{i}(j), ' ', ''), ...
                strrep(knownDates{i}, ' ', ''));
        end
        %Return true if any of the dates matches
        matches(i) = max(multiMatches);
    end
end

%Find accuracy bycalculating ratio between matches and total number of images
accuracy = sum(matches) / imageCount;

%Output to command window
fprintf("Overall Detection & Recognition:\nAccuracy from %.0f images: %.4f\n\n", ...
    imageCount, accuracy);

%% Calculate algorithm execution time

%Calculate the mean, min, max and standared deviation from the execution
%times
avgTime = mean(times);
minTime = min(times);
maxTime = max(times);
stdTime = std(times);

%Output results to command window
fprintf("Expiry Date Algorithm Time Efficiency:\nMinimum: %.4f  Maximum: %.4f\nAverage: %.4f\nStandard Deviation: %.4f\n\n", ...
    minTime, maxTime, avgTime, stdTime);
