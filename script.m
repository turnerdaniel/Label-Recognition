%% MATLAB Script for the recognition of Food Expiry Dates from Images
%Author: Daniel Turner
%University: University of Lincoln
%Date: 23/1/2019

%Stroke Width Transform Algorithm:
%Li, Y. and Lu, H. (2012) Scene Text Detection via Stroke Width. In:
%21st International Conference on Pattern Recognition (ICPR 2012),
%Tsukuba, Japan, 11-15 November. IEEE, 681–684. Available from 
%https://ieeexplore.ieee.org/document/6460226 [accessed 10/2/2019].

%Candidate Text Grouping Algorithm:
%Mathworks (undated) Automatically Detect and Recognize Text in Natural
%Images. Available from https://uk.mathworks.com/help/vision/examples/
%automatically-detect-and-recognize-text-in-natural-images.html#d120e277 
%[accessed 10 February 2019].

%Reset MATLAB environement
clear; close all; clc;

%TODO:
%Implement OCR
%Implement date recognition
%Further false-positive reductions
%Variable Renaming
%Parameter Tweaking
%Optimisation (if/loops/memory)

%Saturday Notes:
%The skeleton implementation is good if referenced (MATLAB & journal)
%   & effecient
%The CC method does have slight disadvanatges (sometimes wrong, thinning,
%   more difficult to destinguish from non-text, interfere w/ barcode)

%Sunday Notes:
%Ensure the citations are jusitified in the report
%Maybe create horz & vert aspect Ratio for more fine-tuning if needed

%Monday Notes:
%Potential improvements include making bbox width of 1 letter (not 1/2)
%SWT Threshold needs to be tested depending on whether we want to preserve
%as much text as possible or leave just the expiry date.

%% Read image

%imgs = ('img/20 NOV(1184)(2325).jpeg');
%       ('img/25 MAR(2354).jpeg');
%       ('img/10 MAR(1820).jpeg');
%       ('img/image1 2 3 4 5 6 7 8.jpeg');
%       ('img/370 378 988.jpeg');
I = imread('img/20 NOV(1184).jpeg');

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

%Perform Linear Contrast Stretching (Could change Gamma?)
greyContrastStretch = imadjust(greyWeiner);

%Use unsharp masking to increase image sharpness
greySharp = imsharpen(greyContrastStretch);

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
opened = imopen(mserBW, seSquare);

figure, subplot(1,2,1), imshow(mserBW), title('original');
subplot(1,2,2), imshow(opened), title('Opened');

%Remove small blobs
clearNoise = bwareaopen(opened, 100); 
%Close small holes by inverting image between foreground and background
clearSmallHoles = ~bwareaopen(~clearNoise, 3);
figure, imshow(clearSmallHoles), title('No holes & Small Blobs');

%% Connected Component Enhanced MSER (CCEMSER)

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
    
    %theshold the cropped greyscale image using Otsu's Method
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

%Calculte the maximum horizontal/vertical aspect ratio
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

swtStats = regionprops(keptObjectsImage, 'Image');
swtLabel = bwlabel(keptObjectsImage);

totalObjects = size(swtStats, 1);
swtVariation = zeros(1, totalObjects);

%Lowest = 0.375
swtVariationThresh = 0.4;

tic
for i = 1:totalObjects
    
    object = swtStats(i).Image;
       
    %Pad the image to avoid boundary effects
    paddedObject = padarray(object, [1 1]);
        
    %Distance Transform
    distanceTransform = bwdist(~paddedObject);
    %Thinning until Skeleton
    skeletonTransform = bwmorph(paddedObject, 'thin', inf);
        
    %Retrieve the stroke width values for the image
    swtWidths = distanceTransform(skeletonTransform);
    
    %Calculate the variation in stroke widths
    swtVariation(i) = std(swtWidths)/mean(swtWidths);
end
loopTime = toc

validStrokeWidths = swtVariation < swtVariationThresh;
keptSWT = find(validStrokeWidths);
keptSWTImage = ismember(swtLabel, keptSWT);

figure, imshow(keptSWTImage), title('Filter images using SWT');
%figure, plot(swtVariation), yline(swtVariationThresh); title('SW Variation in Image');

%Get MSER regions within the ROI bbox?
%Need to use MSER cc regions from the start / could just do for this step?

%% Rule-Based Candidate Text Grouping

%Display potential text regions
textStats = regionprops(keptSWTImage, 'BoundingBox');
textROI = vertcat(textStats.BoundingBox);
textROIImage = insertShape(I, 'Rectangle', textROI, 'LineWidth', 2);
figure, imshow(textROIImage), title('Text ROI''s');

% textSe = strel('line', 25, 0);
% text = imclose(keptSWTImage, textSe);
% regionStats = regionprops(text, 'BoundingBox');
% dilatedROI = vertcat(regionStats.BoundingBox);
% dilatedTextROIImage = insertShape(I, 'Rectangle', dilatedROI, 'LineWidth', 2);
% figure, imshow(dilatedTextROIImage), title('Expanded Text ROI');

%Get bounding box sizes
x = textROI(:, 1);
y = textROI(:, 2);
w = textROI(:, 3);
h = textROI(:, 4);

%Expand ROI by half the character width in horizontal direction since dates
%are always vertically aligned
expandedX = x - (w/2);
expandedW = 2 * w;

%Ensure that ROI is within bounds of the image
expandedX = max(expandedX, 1);
expandedW = min(expandedW, imageWidth - expandedX);

%Create expanded bounding boxes
expandedTextROI = [expandedX, y, expandedW, h];
expandedTextROIImage = insertShape(I, 'Rectangle', expandedTextROI, 'LineWidth', 2);
figure, imshow(expandedTextROIImage), title('Expanded Text ROI');

%Calculate the ratio of union between bounding boxes
overlapRatio = bboxOverlapRatio(expandedTextROI, expandedTextROI, 'Union');
overlapSize = size(overlapRatio, 1);

%Remove union with own Bounding Box
overlapRatio(1:overlapSize + 1:overlapSize^2) = 0;

%Create node graph of connected Bounding Boxes & find the index of boxes
%that intersect
[labelledROI, labelSizes] = conncomp(graph(overlapRatio));

tic
%Ensure that there is similarity between connected letters
maxComponents = max(labelledROI);
%loop through connected bounding boxes
for k = 1:maxComponents
    %find the index of connected bounding boxes
    connectedBoxes = find(labelledROI == k);
    %get the bounding box heights
    heightOfBoxes = h(connectedBoxes);
    
    %Ensure that joined regions have similar heights
    meanHeight = mean(heightOfBoxes);
    meanError = meanHeight/2;
    
    %create mask of bounding boxes that don't meet the criteria
    validBoxes = heightOfBoxes > meanHeight - meanError & heightOfBoxes < meanHeight + meanError;
    discardHeight = find(validBoxes == 0);
    
    %get the index of bounding boxes that don't meet the criteria
    invalidHeight = connectedBoxes(discardHeight);
    
    %Check that there the validHeights matrix is not empty
    if (~isempty(invalidHeight))
        %loop through invalid indexes
        for i = 1:size(invalidHeight, 2)
            id = invalidHeight(i);
            %Seperate the component into a new indices by creating
            %a new 'label' above max value
            labelledROI(id) = max(labelledROI) + 1;
            %Add new value to the end of component size
            labelSizes(size(labelSizes, 2) + 1) = 1;
        end
    end
end
loopTime = toc

%Find the minimum values of x & y and the maximum values of w & h for each 
%of the intersecting bounding boxes to form encompassing bounding boxes
labelledROI = labelledROI';
x1 = accumarray(labelledROI, expandedX, [], @min);
y1 = accumarray(labelledROI, y, [], @min);
x2 = accumarray(labelledROI, expandedX + expandedW, [], @max);
y2 = accumarray(labelledROI, y + h, [], @max);

%Create merged bounding boxes in appropriate format
mergedTextROI = [x1, y1, x2 - x1, y2 - y1];
mergedTextROIImage = insertShape(I, 'Rectangle', mergedTextROI, 'LineWidth', 2);
figure, imshow(mergedTextROIImage), title('Merged Text ROI of Similar Size');

%Remove single, unconnected bounding boxes
wordCandidates = labelSizes > 1;
filteredTextROI = mergedTextROI(wordCandidates, :);

%Remove bounding boxes that are now empty from being seperated
validSize = sum(filteredTextROI, 2) > 0;
filteredTextROI = filteredTextROI(validSize, :);

filteredTextROIImage = insertShape(I, 'Rectangle', filteredTextROI, 'LineWidth', 2);
figure, imshow(filteredTextROIImage), title('Remove Singular ROI');

%Expand a little vertically to fully contain the height of each word
pixels = 2;
expandedY = filteredTextROI(:, 2) - pixels;
expandedH = filteredTextROI(:, 4) + (2 * pixels);
%Ensure that ROI is within bounds of the image
expandedY = max(expandedY, 1);
expandedH = min(expandedH, imageHeight - expandedH);

%Create expanded bounding boxes in appropriate format
expandedFilteredTextROI = [filteredTextROI(:, 1), expandedY, ...
    filteredTextROI(:, 3), expandedH];

expandedFilteredTextROIImage = insertShape(I, 'Rectangle', expandedFilteredTextROI, 'LineWidth', 2);
figure, imshow(expandedFilteredTextROIImage), title('Expand ROI');

%% Perform Optical Character Recognition (OCR) & Preperation

ocrROI = [expandedFilteredTextROI(:, 1), expandedFilteredTextROI(:, 2), ...
    expandedFilteredTextROI(:, 3), expandedFilteredTextROI(:, 4)];

ROISize = size(ocrROI, 1);

detectedText = strings(ROISize, 1);
%Loop through candidate words
for i = 1:ROISize
    ROI = imcrop(keptSWTImage, [ocrROI(i, 1), ocrROI(i, 2), ocrROI(i, 3), ...
        ocrROI(i, 4)]);
    
    ROI = imclearborder(ROI);
    
    stats = regionprops(ROI, 'BoundingBox');
    
    centreX = zeros(1, size(stats, 1));
    centreY = zeros(1, size(stats, 1));
    
    figure, imshow(ROI);
    %Loop through letters
    for j = 1:size(stats, 1)
        bb = stats(j).BoundingBox;
        centreX(j) = round(mean([bb(1), bb(1)+bb(3)]));
        centreY(j) = round(mean([bb(2), bb(2)+bb(4)]));
        hold on;
        plot(centreX, centreY, 'b+', 'MarkerSize', 5, 'LineWidth', 2);
        hold off;
    end
    
    coeff = polyfit(centreX, centreY, 1);
    
    %size = ROI Width
    xFit = linspace(1, size(ROI, 2), 10);
    
    yFit = polyval(coeff, xFit);
    
    angle = atan2d(yFit(10) - yFit(1), xFit(10) - xFit(1));
    
    hold on;
    plot(xFit, yFit, 'g.-', 'MarkerSize', 15, 'LineWidth', 1);
    title([num2str(angle), ' degrees']);
    hold off;
    
    correctedROI = imrotate(ROI, angle);
    
    figure, imshow(correctedROI), title('corrected')
    
    ocrOutput = ocr(correctedROI, 'TextLayout', 'Line', 'CharacterSet', ...
        '1234567890ABCDEFGHIJLMNOPRSTUVYabcdefghijlmnoprstuvy/.-:');
    
    detectedText(i) = ocrOutput.Text;
end

detectedText

%TODO:
%test all images
%stop over-compensating rotation
%only rotate when necessary
%increase image size if necessary
%optimise ocr loop
%investigate regex for character set
%test text grouping


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
%Rotate to minimise BBox size & get correct orienttion
%Get rotation that minimise height of image (top/bottom pixel)
%In case of split, Group together text on same y axis and close x axis
%ROI could be too small - check
%Sort by X/Y to get correct reading order

%% Perform Text Matching using Regex

% May have additional text recognised. eg. descriptions/food title
% eliminate by looking for common data formats:
% DD/MM/YYYY | DD.MM.YYYY | DD MMM or DD JUNE or DD JULY | DD MMMMMMM, etc.
%
% Could also look for dates on y-axis of months of date formats...
% If not found, could repeat ocr with new threshold?

%% Print the Date/Save to File


