%% MATLAB Script for the recognition of Food Expiry Dates from Images
%Author: Daniel Turner
%University: University of Lincoln
%Date: 23/1/2019

%Stroke Width Transform Algorithm:
%Li, Y. and Lu, H. (2012) Scene Text Detection via Stroke Width. In:
%21st International Conference on Pattern Recognition (ICPR 2012),
%Tsukuba, Japan, 11-15 November. IEEE, 681�684. Available from 
%https://ieeexplore.ieee.org/document/6460226 [accessed 10 February 2019].

%Candidate Text Grouping Algorithm:
%Mathworks (undated) Automatically Detect and Recognize Text in Natural
%Images. Available from https://uk.mathworks.com/help/vision/examples/
%automatically-detect-and-recognize-text-in-natural-images.html#d120e277 
%[accessed 10 February 2019].

%Reset MATLAB environement
clear; close all; clc;

%Disable non-criticial warnings for image resizing
warning('off', 'images:initSize:adjustingMag'); 

%% Read Ground Truths

%Warning: you will need to change the local filepaths for the dataset in the 
%ground truth object using changeFilePaths() 
load('gTruths.mat');

%Get size of dataset
imageCount = size(gTruth.LabelData, 1);

%Initialise precision and recall vectors
precision = zeros(imageCount, 1);
recall = zeros(imageCount, 1);

%% Run Script 
tic
for val = 1:imageCount
    %% Read image 
    I = imread(gTruth.DataSource.Source{val});
    
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

    %Display pre-processing effects
    %figure, subplot(2,2,1), imshow(grey), title('Greyscale Image');
    %subplot(2,2,2), imshow(greyWeiner), title('Linear Weiner Filter');
    %subplot(2,2,3), imshow(greyContrastStretch), title('Contrast Stretching');
    %subplot(2,2,4), imshow(greySharp), title('Unsharp Masking');

    %% Maximally Stable Extremal Regions (MSER)

    %Detect MSER Regions
    %RegionAreaRange = region min|max size (30 14000)
    %ThresholdDelta = Step size between intensity threshold (2)
    %MaxAreaVariation = max area variation between regions (0.25)
    mserRegions = detectMSERFeatures(greySharp, 'RegionAreaRange', ... 
        [150 1500], 'ThresholdDelta', 2.5, 'MaxAreaVariation', 0.2);

    %Concatenate pixel coordinates as Nx2 matrix
    mserPixels = vertcat(cell2mat(mserRegions.PixelList));

    %Display MSER Regions overlay on image
    %figure, imshow(I);
    %hold on;
    %plot(mserRegions, 'showPixelList', true, 'showEllipses', false);
    %title('MSER Regions');
    %hold off;

    %Initialise logical image with necessary dimensions
    mserBW = false(imageHeight, imageWidth);
    %Convert img co-ordinates to linear image indexes
    ind = sub2ind(size(mserBW), mserPixels(:,2), mserPixels(:,1));
    %assign true to co-ordinates that match
    mserBW(ind) = true;
    %figure, imshow(mserBW), title('logical MSER Image');

    %% Image Post-Processing - Opening, 

    seSquare = strel('square', 3);
    %seDiamond = strel('diamond', 1);

    %Opening to remove small joins
    opened = imopen(mserBW, seSquare);

    %figure, subplot(1,2,1), imshow(mserBW), title('Original');
    %subplot(1,2,2), imshow(opened), title('Opened');

    %Remove small blobs
    clearNoise = bwareaopen(opened, 100); 
    %Close small holes by inverting image between foreground and background
    clearSmallHoles = ~bwareaopen(~clearNoise, 3);
    %figure, imshow(clearSmallHoles), title('No holes & Small Blobs');

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

    %figure, imshow(CCadjustedImage), title('CC Adjustment');

    %Close small gaps ################################
    %CCadjustedImage = imclose(CCadjustedImage, seSquare);

    %Remove small blobs
    clearNoise = bwareaopen(CCadjustedImage, 20); 
    %Close small holes by inverting image between foreground and background
    clearSmallHoles = ~bwareaopen(~clearNoise, 3);
    %figure, imshow(clearSmallHoles), title('No holes & Small Blobs V2');

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

    % figure, subplot(3,2,1), plot([mserStats.EulerNumber]), title('Euler');
    % subplot(3,2,2), plot([mserStats.Eccentricity]), title('Eccentricity');
    % subplot(3,2,3), plot([mserStats.Extent]), title('Extent');
    % subplot(3,2,4), plot([mserStats.Solidity]), title('Solidity');
    % subplot(3,2,5), plot(aspectRatio), title('Aspect');

    %Find the index of all objects that have valid properties
    keptObjects = find(validEulerNo & validEccentricity & validExtent & ...
        validSolidity & validAspectRatio);
    %Return pixels that satisfy the similar property criteria for the image
    keptObjectsImage = ismember(mserLabel, keptObjects);

    % figure, imshow(clearSmallHoles), title('original');
    % figure, imshow(ismember(mserLabel, find(validEulerNo))), title('Valid Euler No');
    % figure, imshow(ismember(mserLabel, find(validEccentricity))), title('Valid Eccentricity');
    % figure, imshow(ismember(mserLabel, find(validExtent))), title('Valid Extent');
    % figure, imshow(ismember(mserLabel, find(validSolidity))), title('Valid Solidity');
    % figure, imshow(ismember(mserLabel, find(validAspectRatio))), title('Valid Aspect');

    %figure, imshow(keptObjectsImage), title('Filter images using text properties')

    %% Stroke Width Transform

    %Label the image and get the properties of each object
    swtLabel = bwlabel(keptObjectsImage);
    swtStats = regionprops(keptObjectsImage, 'Image');

    %Initialise variables
    totalObjects = size(swtStats, 1);
    swtVariation = zeros(1, totalObjects);

    %Define the maximum variation in stroke width for letters
    %Lowest = 0.375
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

    %figure, imshow(keptSWTImage), title('Filter images using SWT');
    %figure, plot(swtVariation), yline(swtVariationThresh); title('SW Variation in Image');

    %% Rule-Based Candidate Text Grouping

    %Get the bounding box for each object
    textStats = regionprops(keptSWTImage, 'BoundingBox');
    textROI = vertcat(textStats.BoundingBox);
    %Insert the bounding boxes onto the image and display
    textROIImage = insertShape(I, 'Rectangle', textROI, 'LineWidth', 2);
    %figure, imshow(textROIImage), title('Text ROI''s');

    %Get bounding box sizes
    roiX = textROI(:, 1);
    roiY = textROI(:, 2);
    roiW = textROI(:, 3);
    roiH = textROI(:, 4);

    %Expand ROI by half the character width in horizontal direction since dates
    %are typically vertically aligned
    expandedX = roiX - (roiW/2);
    expandedW = 2 * roiW;

    %Ensure that ROI is within bounds of the image
    expandedX = max(expandedX, 1);
    expandedW = min(expandedW, imageWidth - expandedX);

    %Create expanded bounding boxes
    expandedTextROI = [expandedX, roiY, expandedW, roiH];
    expandedTextROIImage = insertShape(I, 'Rectangle', expandedTextROI, 'LineWidth', 2);
    %figure, imshow(expandedTextROIImage), title('Expanded Text ROI');

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
    for k = 1:maxComponents
        %find the index of connected bounding boxes
        connectedBoxes = find(labelledROI == k);
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
            for i = 1:size(invalidHeight, 2)
                id = invalidHeight(i);
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
    mergedTextROIImage = insertShape(I, 'Rectangle', mergedTextROI, 'LineWidth', 2);
    %figure, imshow(mergedTextROIImage), title('Merged Text ROI of Similar Size');

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

    filteredTextROIImage = insertShape(I, 'Rectangle', filteredTextROI, 'LineWidth', 2);
    %figure, imshow(filteredTextROIImage), title('Remove Singular ROI');

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

    expandedFilteredTextROIImage = insertShape(I, 'Rectangle', expandedFilteredTextROI, 'LineWidth', 2);
    %figure, imshow(expandedFilteredTextROIImage), title('Expand ROI');
    
    %% Calculate Precision and Recall
    
    gTruthImage = insertShape(I, 'Rectangle', gTruth.LabelData{val, 1}{1}, 'LineWidth', 2, 'Color', 'green');
    figure, subplot(1,2,1), imshow(expandedFilteredTextROIImage);
    subplot(1,2,2), imshow(gTruthImage);
    
    %Show progress
    disp(val);
    
    %calculate precision and recall
    [precision(val), recall(val)] = bboxPrecisionRecall(expandedFilteredTextROI, gTruth.LabelData{val, 1}{1});

end
runTime = toc