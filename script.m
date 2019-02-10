%% MATLAB Script for the recognition of Expiry Dates from Images
%Reset MATLAB environement
clear; close all; clc;

%TODO:
%Fine-tune Region property Segmentation
%Implement SWT/Gabor/K-Means 
%Improve OCR
%Implement date recognition
%Further false-positive reductions
%Variable Renaming

%Saturday Notes:
%The skeleton implementation is good if referenced (MATLAB & journal) & effecient
%Optimise if statements & loop
%Investigate SWT threshold
%The CC method does have slight disadvanatges (sometimes wrong, thinning,
%   more difficult to destinguish from non-text, interfere w/ barcode)

%Sunday:
%Test CC - check aspect ratio/soldity parameters for 20 NOV(2325)
%Do above
%Implement BBox joining w/ rules
%Add code references & clean up vars
%Check ROI Rules

%% Read image

%imgs = ('img/20 NOV(1184)(2325).jpeg');
%       ('img/25 MAR(2354).jpeg');
%       ('img/10 MAR(1820).jpeg');
%       ('img/image1 2 3 4 5 6 7.jpeg');
%       ('img/370 378 988.jpeg');
I = imread('img/image6.jpeg');

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

%use imbothat w/ large strel to remove uneven illumination?
%deblurring?
%Orientation Correction?

%Perform Linear Spatial Filtering to eliminate noise
%Weiner removes gaussian & speckle noise while preserving edges by adapting
%smoothing amount
greyWeiner = wiener2(grey, [3 3]);

%Salt & Pepper noise not common on digital images
%greyMed = medfilt2(grey, [3 3]);

%Perform Linear Contrast Stretching (Could change Gamma?)
greyContrastStretch = imadjust(greyWeiner);

%Originally used CLAHE but that introduced more noise and increased size of
%letters; leading to joining
%greyClahe = adapthisteq(greyWeiner);

%Use unsharp masking to increase image sharpness
greySharp = imsharpen(greyContrastStretch);

%Laplacian/unsharp mask sharpening produce very similar results
%Laplacian more noisy on some tests but neither of them have clear
%advantages

%Display pre-processing effects
figure, subplot(2,2,1), imshow(grey), title('Greyscale Image');
subplot(2,2,2), imshow(greyWeiner), title('Linear Weiner Filter');
subplot(2,2,3), imshow(greyContrastStretch), title('Contrast Stretching');
subplot(2,2,4), imshow(greySharp), title('Unsharp Masking');

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
figure, imshow(I);
hold on;
plot(mserRegions, 'showPixelList', true, 'showEllipses', false);
title('MSER Regions');
hold off;

%Initialise logical image with necessary dimensions
mserBW = false(imageHeight, imageWidth);
%Convert img co-ordinates to linear image indexes
ind = sub2ind(size(mserBW), mserPixels(:,2), mserPixels(:,1));
%assign true to co-ordinates that match
mserBW(ind) = true;
figure, imshow(mserBW), title('logical MSER Image');

%% Image Post-Processing - Opening, 
%Watershed may be good?
%bwmorph for thinning?

seSquare = strel('square', 3);
%seDiamond = strel('diamond', 1);

%Opening to remove small joins
%Maybe adjust to small se?
opened = imopen(mserBW, seSquare);

figure, subplot(1,2,1), imshow(mserBW), title('original');
subplot(1,2,2), imshow(opened), title('Opened');

%Remove small blobs
clearNoise = bwareaopen(opened, 100); 
%Close small holes by inverting image between foreground and background
clearSmallHoles = ~bwareaopen(~clearNoise, 3);
figure, imshow(clearSmallHoles), title('No holes & Small Blobs');

%% Connected Component Enhanced MSER (CCEMSER)
%Intersection with threshold like: https://ieeexplore.ieee.org/document/7760054

mserStats = regionprops(clearSmallHoles, 'Image', 'BoundingBox');
totalObjects = size(mserStats, 1);
CCadjustedImage = false(imageHeight, imageWidth);

tic
for i = 1:totalObjects
    %Get BBox and image
    %Correct for slightly large BBox and round floating values
    imageBBox = ceil(mserStats(i).BoundingBox - [0, 0, 1, 1]);
    image = mserStats(i).Image;
    
    %Crop the greyscale image
    greyImage = imcrop(greySharp, imageBBox);
    
    %theshold the cropped greyscale image using Otsu
    %Problem is that text can be light/dark (apply twice?)
    binaryImage = imbinarize(greyImage);  
    
    %Perform CC analysis to find most likely text (BoW, DoL text)
    %Each letter should have the lowest number of non-zero components
    ccReg = bwconncomp(image & binaryImage);
    ccInv = bwconncomp(image & ~binaryImage);
    
    %Concatenate list to handle multiple objects in image
    ccRegSize = size(cat(1, ccReg.PixelIdxList{:}), 1);
    ccInvSize = size(cat(1, ccInv.PixelIdxList{:}), 1);
    
    %Optimise if statements
    if ccReg.NumObjects == 0 && ccInv.NumObjects ~= 0
        %Use regular image
        keepImage = image & ~binaryImage;
    elseif ccInv.NumObjects == 0 && ccReg.NumObjects ~= 0
        %Use regular image
        keepImage = image & binaryImage;
    elseif ccReg.NumObjects < ccInv.NumObjects
        %Use regular image
        keepImage = image & binaryImage;
    elseif ccInv.NumObjects < ccReg.NumObjects
        %Use regular image
        keepImage = image & ~binaryImage;
    elseif ccInv.NumObjects == ccReg.NumObjects
        if ccRegSize < ccInvSize 
            keepImage = image & ~binaryImage;
        elseif ccInvSize < ccRegSize
            keepImage = image & binaryImage;
        end
    else
        %use regular image as last resort
        keepImage = image & binaryImage;
    end
    
    %Replace existing image locations with new enhanced images
    CCadjustedImage(imageBBox(2):imageBBox(2) + imageBBox(4), ...
        imageBBox(1):imageBBox(1) + imageBBox(3)) = keepImage;    
end
loopTime = toc

figure, imshow(CCadjustedImage), title('CC Adjustment');

%Remove small blobs
clearNoise = bwareaopen(CCadjustedImage, 20); 
%Close small holes by inverting image between foreground and background
clearSmallHoles = ~bwareaopen(~clearNoise, 3);
figure, imshow(clearSmallHoles), title('No holes & Small Blobs V2');

%% Remove Unlikely Candidates using Region Properties

% Eccentricity = similarity to line segment (1) or cirlce (0)
% Euler Number = Don't have many holes (max 2 | B)
% Aspect Ratio = mostly square (vertical & horizontal)
% Extent = have very high or very low occupation of bounding box (O vs l)

mserLabel = bwlabel(clearSmallHoles);
mserStats = regionprops(clearSmallHoles, 'BoundingBox', 'Eccentricity', ...
    'EulerNumber', 'Extent', 'Solidity');

bBoxes = vertcat(mserStats.BoundingBox);
bbWidths = bBoxes(:, 3)';
bbHeights = bBoxes(:, 4)';
aspectRatio = max(bbWidths ./ bbHeights, bbHeights ./ bbWidths);

%Max euler = -1. However, is affected by noise so change to -2
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

%Attempted to use compactness and circulairty but would remove important
%details and thresholds had to be reduced to the point of useless-ness.

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

figure, imshow(keptObjectsImage), title('Filter images using text properties')

%% Stroke Width Transform
%See what changes to threshold do

mserStats = regionprops(keptObjectsImage, 'Image');
mserLabel = bwlabel(keptObjectsImage);

% https://cs.adelaide.edu.au/~yaoli/wp-content/publications/icpr12_strokewidth.pdf
totalObjects = size(mserStats, 1);
variation = zeros(1, totalObjects);

variationThresh = 0.4;

%Attempted to use minimun variaton to eliminate holes in the middle of
%letters to permit accurate SWT. However, meant that SWT wasn't recorded
%correctly due to NaN and 0 being present in std(SWT)/mean(SWT)

%The CCEMSER was needed to ensure that letters weren't filled & therfore
%SWT would be more accurate/less false positives

tic
for i = 1:totalObjects
    
    croppedImage = mserStats(i).Image;
       
    %Pad the image to avoid boundary effects
    paddedImage = padarray(croppedImage, [1 1]);
        
    %Distance Transform
    distanceTransform = bwdist(~paddedImage);
    %Thinning
    skeletonTransform = bwmorph(paddedImage, 'thin', inf);
        
    %Create stroke width image from distanceTransform and skeleton
    %Change to .*
    strokeWidth = distanceTransform(skeletonTransform);
    %Calculate the variation in stroke widths
    variation(i) = std(strokeWidth)/mean(strokeWidth);
end
loopTime = toc

validStrokeWidths = variation < variationThresh;
keptSWT = find(validStrokeWidths);
keptSWTImage = ismember(mserLabel, keptSWT);

figure, imshow(keptSWTImage), title('Filter images using SWT');
figure, plot(variation), yline(variationThresh); title('SW Variation in Image');

% helperStrokeWidth() - or could make own... 
% Pseudocode @ Mathworks and journals

%Get MSER regions within the ROI bbox?
%Need to use MSER cc regions from the start / could just do for this step?

%% Gabor Filters/K-Means Clustering/Watershed/Heatmap

% Additional research required...
% See proposal for K-Means method

%% Detected Text

%Display potential text regions
stats = regionprops(keptSWTImage, 'BoundingBox');
textROI = vertcat(stats.BoundingBox);
textROIImage = insertShape(I, 'Rectangle', textROI, 'LineWidth', 2);

figure, imshow(textROIImage), title('Text ROI''s');

% textSe = strel('line', 25, 0);
% text = imclose(keptSWTImage, textSe);
% regionStats = regionprops(text, 'BoundingBox');
% dilatedROI = vertcat(regionStats.BoundingBox);
% dilatedTextROIImage = insertShape(I, 'Rectangle', dilatedROI, 'LineWidth', 2);
% figure, imshow(dilatedTextROIImage), title('Expanded Text ROI');
%
% Tried to use closing to combine the text into lines. However, it
% resulted in being difficult to respond to the different font sizes.
% Making it hard to capture the entire date without
% over-expanding/under-expanding


%MATLAB reference & why
%https://uk.mathworks.com/help/vision/examples/automatically-detect-and-recognize-text-in-natural-images.html#d120e277

%Expand horizontally - dates will always be on the same line
%Expand by width of of letter, group by similar heights

%Get bounding box sizes
x = textROI(:,1);
y = textROI(:,2);
w = textROI(:,3);
h = textROI(:,4);

%Expand ROI by half the character width in horizontal direction
x = x - (w/2);
w = 2 * w;

%Ensure that BBox is within bounds of the image
x = max(x, 1);
w = min(w, imageWidth - x);

%Create expanded bounding boxes
expandedTextROI = [x, y, w, h];
expandedTextROIImage = insertShape(I, 'Rectangle', expandedTextROI, 'LineWidth', 2);
figure, imshow(expandedTextROIImage), title('Expanded Text ROI');

%Calculate the ratio of union between bounding boxes
overlapRatio = bboxOverlapRatio(expandedTextROI, expandedTextROI, 'Union');
overlapSize = size(overlapRatio, 1);

%Remove union with own Bounding Box
overlapRatio(1:overlapSize + 1:overlapSize^2) = 0;

%Create node graph of connected Bounding Boxes
g = graph(overlapRatio);
%Find the index of boundary boxes that intersect
[componentIndices, componentSizes] = conncomp(g);

%Ensure that there is similarity between connected letters
maxComponentId = max(componentIndices);
%loop through connected bounding boxes
for k = 1:maxComponentId
    %find the index of connected bounding boxes
    connectedBoxes = find(componentIndices == k);
    %get the bounding box heights
    heightOfBoxes = h(connectedBoxes);
    
    %Ensure that joined regions have similar heights
    %###MAYBE CHANGE###%
    meanVal = mean(heightOfBoxes);
    error = meanVal/2;
    
    %create mask of bounding boxes that don't meet the criteria
    validBoxes = heightOfBoxes > meanVal - error & heightOfBoxes < meanVal + error;
    discardHeight = find(validBoxes == 0);
    
    %get the index of bounding boxes that don't meet the criteria
    invalidHeight = connectedBoxes(discardHeight);
    
    %Check that there the validHeights matrix is not empty
    if (~isempty(invalidHeight))
        %loop through invalid indexes
        for i = 1:size(invalidHeight, 2)
            id = invalidHeight(i)
            %Seperate the component into a new indices by essentially
            %creating a new 'label' above max value
            componentIndices(id) = max(componentIndices) + 1;
            %Add new value to the end of component size
            componentSizes(size(componentSizes, 2) + 1) = 1;
        end
    end
end

%Find the minimum values of x & y and the maximum values of w & h for each 
%of the connected components to form merged bounding boxes
componentIndices = componentIndices';
x1 = accumarray(componentIndices, x, [], @min);
y1 = accumarray(componentIndices, y, [], @min);
x2 = accumarray(componentIndices, x + w, [], @max);
y2 = accumarray(componentIndices, y + h, [], @max);

%Create merged bounding boxes in appropriate format
textBBoxes = [x1, y1, x2 - x1, y2 - y1];
mergedTextRegion = insertShape(I, 'Rectangle', textBBoxes, 'LineWidth', 2);
figure, imshow(mergedTextRegion), title('Merged Text Regions');

%Remove single unconnected bounding boxes
wordCandidates = componentSizes > 1;
textBBoxes = textBBoxes(wordCandidates, :);

filteredTextRegion = insertShape(I, 'Rectangle', textBBoxes, 'LineWidth', 2);
figure, imshow(filteredTextRegion), title('Remove Singular Regions');

%Maybe grow only horizontally? Dates aren't vertical.
%Rule-based? grow by own size, etc.
%Rotate to minimiise BBox size & get correct orienttion
%Get rotation that minimise height of image (top/bottom pixel)
%One BBox by self isn't worth

%% Perform Optical Character Recognition (OCR) & Preperation

detectedText = ocr(greySharp, textROI);
[detectedText.Text]

%could perform on multiple images (MSERRegions/greyscale/adaptiveThreshold)?

%improve by creating a ocr characterSet to limit the possible characters to
%letters/numbers found in dates (1234567890 abcdefghij_lmnop_rstuv__y_ /.)

%Change 'textLayout' to 'Block'/'line' may help
%Perform morphology to help thin-out connected characters
%Make sure letters are of adequate size (>20px)
%Use ROI for OCR
%Look at automatic thrshold methods (Gaussian) or threshold for each blob
%May need to get the OCR support package (visionSupportPackages) or 
%download .traineddata from GitHub for best accuracy
%Could use MSER Regions or regions with least connected components
%Need to consider rotation (Hough transform? regionprops.Orientation?)
%Rotate several times and take highest confidence
%OCR w/ Temporal Fusion (takes OCR across a range of different frames)

%% Perform Text Matching using Regex

% May have additional text recognised. eg. descriptions/food title
% eliminate by looking for common data formats:
% DD/MM/YYYY | DD.MM.YYYY | DD MMM or DD JUNE or DD JULY | DD MMMMMMM, etc.
%
% Could also look for dates on y-axis of months of date formats...
% If not found, could repeat ocr with new threshold?

%% Print the Date/Save to File


