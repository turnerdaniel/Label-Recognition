%% MATLAB Script for the recognition of Food Expiry Dates from Images
%Author: Daniel Turner
%University: University of Lincoln
%Date: 23/1/2019

%References:
%Stroke Width Transform Algorithm:
%Li, Y. and Lu, H. (2012) Scene Text Detection via Stroke Width. In:
%21st International Conference on Pattern Recognition (ICPR 2012),
%Tsukuba, Japan, 11-15 November. IEEE, 681–684. Available from 
%https://ieeexplore.ieee.org/document/6460226 [accessed 10 February 2019].

%Candidate Text Grouping Algorithm:
%Mathworks (undated) Automatically Detect and Recognize Text in Natural
%Images. Available from https://uk.mathworks.com/help/vision/examples/
%automatically-detect-and-recognize-text-in-natural-images.html#d120e277 
%[accessed 10 February 2019].

%% Setup Script Environment

%Reset MATLAB environment
clear; close all; clc;

%Disable non-criticial warnings for image resizing
warning('off', 'images:initSize:adjustingMag'); 


%% Read image

%Open UI menu to choose image from file
[inFile, inPath] = uigetfile('samples/*.jpeg', 'Select An Image');

%Check that an image has been selected
if (~isequal(inFile, 0))
    %Handle invalid file types
    try
        %Read the image
        I = imread(fullfile(inPath, inFile));
    catch 
        %Alert user that the image could not be read (wrong filetype)
        fprintf("Unable to read image: %s\n\n", inFile);
        %Exit script
        return
    end
else
    %Alert user that they didn't select a file so the program can't be
    %executed
    fprintf("No image was selected. Exiting...\n\n");
    %Exit script
    return
end

%% Greyscale Conversion
%Check if image is RGB denoted by being 3D array
if size(I,3) > 0
    %Convert to greyscale
    grey = rgb2gray(I);
else
    %Rename since already greyscale
    grey = I;
end

%Get dimensions of image
[imageHeight, imageWidth] = size(grey);

%% Image Pre-processing - Noise Reduction, Increase Contrast & Sharpness

%Perform Spatial Filtering to eliminate noise
%Weiner removes gaussian & speckle noise while preserving edges by adapting
%smoothing amount for 3x3 neighbourhood
greyWeiner = wiener2(grey, [3 3]);

%Perform Linear Contrast Stretching
greyContrastStretch = imadjust(greyWeiner);

%Use Unsharp Masking to increase image sharpness
greySharp = imsharpen(greyContrastStretch);

%Display output of pre-processing stage
figure, imshow(greySharp), title('Pre-processed Image');

%% Maximally Stable Extremal Regions (MSER)

%Detect text using MSER
%RegionAreaRange = region min/max size (default: 30 14000)
%ThresholdDelta = Step size between intensity threshold (default: 2)
%MaxAreaVariation = max area variation between regions (default: 0.25)
mserRegions = detectMSERFeatures(greySharp, 'RegionAreaRange', ... 
    [150 1500], 'ThresholdDelta', 2.5, 'MaxAreaVariation', 0.2);

%Concatenate pixel coordinates from MSER as Nx2 matrix
mserPixels = vertcat(cell2mat(mserRegions.PixelList));

%Initialise logical image with necessary dimensions
mserBW = false(imageHeight, imageWidth);
%Convert img co-ordinates to linear image indexes
ind = sub2ind(size(mserBW), mserPixels(:,2), mserPixels(:,1));
%Set matching pixels to white
mserBW(ind) = true;

%Display output of MSER as binary mask
figure, imshow(mserBW), title('logical MSER Image');

%% Image Post-Processing - Opening, Removal of Small Objects, Region Filling

%Define 3x3 square structuring element
seSquare = strel('square', 3);

%Perform opening to remove protrusions and small joins
opened = imopen(mserBW, seSquare);

%Remove blobs with an area below 100 pixels
clearNoise = bwareaopen(opened, 100); 
%Fill small holes by inverting image so the background becomes foreground
clearSmallHoles = ~bwareaopen(~clearNoise, 3);

%Display output of post-processing stage
figure, imshow(clearSmallHoles), title('Post-Processed Image');

%% Connected Component Enhanced MSER (CCEMSER)

%Get the image solely containing each object and its bounding box
mserStats = regionprops(clearSmallHoles, 'Image', 'BoundingBox');
%Initialise variables to hold total number of objects and output image
totalObjects = size(mserStats, 1);
CCadjustedImage = false(imageHeight, imageWidth);

%Loop through all objects in image
for i = 1:totalObjects
    %Get bounding box and image for current object
    %Correct for slightly large bbox by rounding co-ordinates
    imageBBox = ceil(mserStats(i).BoundingBox - [0, 0, 1, 1]);
    image = mserStats(i).Image;
    
    %Crop the greyscale image
    greyImage = imcrop(greySharp, imageBBox);
    
    %Threshold the cropped greyscale image using Otsu's Method
    binaryImage = imbinarize(greyImage);  
    
    %Identify connected components in the intersection between binary image
    %and thresholded image
    ccReg = bwconncomp(image & binaryImage);
    ccInv = bwconncomp(image & ~binaryImage);
    
    %Concatenate list to handle multiple objects in an image
    ccRegSize = size(cat(1, ccReg.PixelIdxList{:}), 1);
    ccInvSize = size(cat(1, ccInv.PixelIdxList{:}), 1);
    
    %Check objects to find which has the fewest number of components above 0 which
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
        %Choose threshold that produces the most pixels
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

%Remove blobs with an area below 100 pixels
clearNoise = bwareaopen(CCadjustedImage, 20); 
%Fill small holes by inverting image so the background becomes foreground
clearSmallHoles = ~bwareaopen(~clearNoise, 3);

%Display output of CCEMSER stage
figure, imshow(clearSmallHoles), title('Connected-Componenent Enhanced Image');

%% Geometric Filtering

%Label the image and get the properties of each object
mserLabel = bwlabel(clearSmallHoles);
mserStats = regionprops(clearSmallHoles, 'BoundingBox', 'Eccentricity', ...
    'EulerNumber', 'Extent', 'Solidity');

%Calculte the maximum aspect ratio for horizontal and vertical direction
bBoxes = vertcat(mserStats.BoundingBox);
bbWidths = bBoxes(:, 3)';
bbHeights = bBoxes(:, 4)';
aspectRatio = max(bbWidths ./ bbHeights, bbHeights ./ bbWidths);

%Keep blobs that have less than 3 holes in them. Need to compensate for noise
validEulerNo = [mserStats.EulerNumber] >= -2;
%Keep blobs that are not perfect lines (eg. barcodes)
validEccentricity = [mserStats.Eccentricity] < 0.99;
%Keep blobs that fall within normal distribution of Area to BBox ratio
validExtent = [mserStats.Extent] > 0.25 & [mserStats.Extent] < 0.9;
%Keep blobs that do not have significant convex areas
validSolidity = [mserStats.Solidity] > 0.4;
%Keep blobs that are not extremely elongated 
validAspectRatio = aspectRatio < 3;

%Find the index of all objects that have valid properties
keptObjects = find(validEulerNo & validEccentricity & validExtent & ...
    validSolidity & validAspectRatio);
%Return pixels that are at the valid indexes for the image
keptObjectsImage = ismember(mserLabel, keptObjects);

%Display output of geometric filtering
figure, imshow(keptObjectsImage), title('Filtered Image using Text Properties')

%% Stroke Width Transform

%Label the image and get the smallet image encapsulating each object
swtLabel = bwlabel(keptObjectsImage);
swtStats = regionprops(keptObjectsImage, 'Image');

%Initialise variables that hold the number of objects and their stroke
%width variations
totalObjects = size(swtStats, 1);
swtVariation = zeros(1, totalObjects);

%Define the maximum variation in stroke width for letters
swtVariationThresh = 0.4;

%Loop through all objects in image
for i = 1:totalObjects
    %Get the image conting just the object
    object = swtStats(i).Image;
       
    %Pad the image with 0's to avoid stroke width being affected by boundary
    paddedObject = padarray(object, [1 1]);
        
    %Perform Distance Transform
    distanceTransform = bwdist(~paddedObject);
    
    %Perform thinning until Skeleton is created
    skeletonTransform = bwmorph(paddedObject, 'thin', inf);
        
    %Retrieve the stroke width values for the image
    swtWidths = distanceTransform(skeletonTransform);
    
    %Calculate the variation in stroke widths
    swtVariation(i) = std(swtWidths)/mean(swtWidths);
end

%Find valid stroke widths that are below the variation threshold
validStrokeWidths = swtVariation < swtVariationThresh;
%Find the index of these objects
keptSWT = find(validStrokeWidths);
%Create an image made of objects that are below the variation threshold
keptSWTImage = ismember(swtLabel, keptSWT);

%Display output from Stroke Width Transform
figure, imshow(keptSWTImage), title('Filtered Image using Stroke Width Transform');
%figure, plot(swtVariation), yline(swtVariationThresh); title('Stroke Width Variation in Image');

%% Rule-Based Text Grouping

%Get the bounding box for each object and convert to usable coordinates
textStats = regionprops(keptSWTImage, 'BoundingBox');
textROI = vertcat(textStats.BoundingBox);
%Insert the bounding boxes onto the image and display
textROIImage = insertShape(I, 'Rectangle', textROI, 'LineWidth', 2);
figure, imshow(textROIImage), title('Text ROI''s');

%Seperate bounding box into seperate variables for manipulation
roiX = textROI(:, 1);
roiY = textROI(:, 2);
roiW = textROI(:, 3);
roiH = textROI(:, 4);

%Expand ROI by 2/3 the character width in horizontal direction 
expandedX = roiX - (roiW * (2/3)); 
expandedW = roiW + ((roiW * (2/3)) * 2);

%Ensure that ROI is within bounds of the image
expandedX = max(expandedX, 1);
expandedW = min(expandedW, imageWidth - expandedX);

%Create expanded bounding boxes
expandedTextROI = [expandedX, roiY, expandedW, roiH];
expandedTextROIImage = insertShape(I, 'Rectangle', expandedTextROI, 'LineWidth', 2);
%Display expanded bounding boxes
figure, imshow(expandedTextROIImage), title('Expanded Text ROI');

%Calculate the ratio of union between bounding boxes
overlapRatio = bboxOverlapRatio(expandedTextROI, expandedTextROI, 'Union');
overlapSize = size(overlapRatio, 1);

%Remove union with own Bounding Box
overlapRatio(1:overlapSize + 1:overlapSize^2) = 0;

%Create node graph of connected Bounding Boxes & find the index of boxes
%that intersect
labelledROI = conncomp(graph(overlapRatio));

%Find the total number of bounding boxes
totalComponents = max(labelledROI);

%loop through connected bounding boxes
for i = 1:totalComponents
    %find the index of connected bounding boxes
    connectedBoxes = find(labelledROI == i);
    %Get their heights
    heightOfBoxes = roiH(connectedBoxes);
    
    %Calculate the average height of connected bounding boxes
    meanHeight = mean(heightOfBoxes);
    meanError = meanHeight/2;
    
    %Find all bounding boxes that have a height that matches the criteria
    validBoxes = heightOfBoxes > meanHeight - meanError & heightOfBoxes < meanHeight + meanError;
    %Get the index of all bounding boxes that don't match the criteria
    discardHeight = find(validBoxes == 0);
    
    %Get bounding boxes that don't meet the criteria using index
    invalidHeight = connectedBoxes(discardHeight);
    
    %Check that invalidHeights is not empty
    if (~isempty(invalidHeight))
        %Loop through invalid indexes
        for j = 1:size(invalidHeight, 2)
            %Get the id of the bounding box with an invalid height
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
mergedTextROIImage = insertShape(I, 'Rectangle', mergedTextROI, 'LineWidth', 2);
%Display merged bounding boxes
figure, imshow(mergedTextROIImage), title('Merged Text ROI of Similar Size');

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
    %Set pixels to 0 inside bounding boxes that need to be removed
    removeROIImage(removeMinY(i):removeMaxY(i), removeMinX(i):removeMaxX(i)) = 0; 
end

%Display bounding boxes after individual regions have been removed
filteredTextROIImage = insertShape(I, 'Rectangle', filteredTextROI, 'LineWidth', 2);
figure, imshow(filteredTextROIImage), title('Remove Singular ROI''s');

%Expand the bounding box vertically to fully contain the text's height 
pixelExpansion = 2;
expandedY = filteredTextROI(:, 2) - pixelExpansion;
expandedH = filteredTextROI(:, 4) + (2 * pixelExpansion);
%Ensure that ROI is within bounds of the image
expandedY = max(expandedY, 1);
expandedH = min(expandedH, imageHeight - expandedH);

%Create expanded bounding boxes in appropriate format
expandedFilteredTextROI = [filteredTextROI(:, 1), expandedY, ...
    filteredTextROI(:, 3), expandedH];

%Display output of Rule-based Text Grouping stage
expandedFilteredTextROIImage = insertShape(I, 'Rectangle', expandedFilteredTextROI, 'LineWidth', 2);
figure, imshow(expandedFilteredTextROIImage), title('Expanded Merged ROI''s');

%% Orientation Correction & Optical Character Recognition (OCR)

%Get text ROIs to perform OCR on
ocrROI = [expandedFilteredTextROI(:, 1), expandedFilteredTextROI(:, 2), ...
    expandedFilteredTextROI(:, 3), expandedFilteredTextROI(:, 4)];

%Pre-allocate vectors & initialise variables to hold number of bboxes, OCR
%results and the sampling points for the line of best fit
ROISize = size(ocrROI, 1);
detectedText = strings(ROISize, 1);
samplePoints = 10;

%Loop through text lines
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
    
    %Get the centre of the bounding boxes. This is better than MATLAB centroids as
    %they calculate true centre
    centreX = round(mean([letterBoxes(:, 1), letterBoxes(:, 1) + letterBoxes(:, 3)], 2));
    centreY = round(mean([letterBoxes(:, 2), letterBoxes(:, 2) + letterBoxes(:, 4)], 2));
    
    %figure, imshow(ROI);
    %hold on; plot(centreX, centreY, 'b+', 'MarkerSize', 5, 'LineWidth', 1); hold off;
    
    %Calculate line of best fit coeffecients using the centre of letters
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
    
    %hold on; plot(xValues, yValues, 'g.-', 'MarkerSize', 10, 'LineWidth', 1), title([num2str(angle), ' degrees']);
    %hold off;
    
    %Don't correct image if it is within 7.5 degrees of 0
    %OCR is capable of reading letters most accurately at ~< 10 degree offset
    if (angle > 7.5 || angle < -7.5)
        %Rotate the image by the angle using nearest negithbour interpolation
        ROI = imrotate(ROI, angle);
        %figure, imshow(ROI), title('corrected')
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

%% Date Matching

%Formats covered by Regex:
%1. DD MMM / DD MMM YY / DD MMM YYYY / DDMMMYY / DDMMMYYYY
%2. MMM YY / MMM YYYY / MMMYY / MMMYYYY / MMM-YY / MMM-YYYY etc.
%3. DD/MM/YY / DD/MM/YYYY / MM/DD/YY / MM/DD/YYYY / YY/MM/DD
%4. DD.MM.YY / DD.MM.YYYY / MM.DD.YY / MM.DD.YYYY / YY.MM.DD
%5. DD-MM-YY / DD-MM-YYYY / MM-DD-YY / MM-DD-YYYY / YY-MM-DD
%6. DD/MMM/YY / DD/MMM/YYYY / DD.MMM.YY / DD.MMM.YYYY / DD-MMM-YY / DD-MMM-YYYY

%Formats not Supported:
%YYYY/MM/DD / YYYY.MM.DD / YYYY-MM-DD = Difficult to differentiate
%between the more popular DD-MM-YYYY format resulting in false positives.
%DDMMYY / DDMMYYYY / DD MM YY/ DD MM YYYY = Can't differentiate from set 
%of numbers found on barcode or the rest of the packaging.

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

%concatenate matching text into single string array
expiryDates = string(vertcat(validTextDate{:}, validTextYear{:}, validNumeric{:}));

%% Print the Dates or Save to File

%Check to see if dates have been detected
if (size(expiryDates, 1) > 0)
    %create message holding the detected dates
    data = expiryDates;
    caption = 'Success';
    message = (['Managed to find the following expiry dates:'; data]);
else
    %create message holding all detected text
    data = detectedText;
    caption = 'Error!';
    message = (['Couldn''t find any expiry dates. Here''s all the text that was found:'; ...
        data]);
end

%Create message box that displays message with option to save to file
buttonPress = questdlg(message, caption, 'Okay', 'Save...', 'Okay');

%Check if save button is pressed
if strcmpi(buttonPress, 'Save...')
        %Present user with 'Save As' dialog box
        [outFile, outPath] = uiputfile([extractBefore(inFile, '.'), '-detections.txt']);
        %Check that the a destination has been selected
        if (~isequal(outFile, 0))
            %Open the file with overwrite permissions
            fileID = fopen(fullfile(outPath, outFile), 'w');
            %Write detected text/dates to file
            fprintf(fileID, '%s\n', data);
            %Close the file
            fclose(fileID);
        else
            %Alert the user that the file wasn't saved
            fprintf("No destination selected. File was not saved.\n\n");
        end
end

%Output dates/text in the command window
fprintf('%s\n', message);