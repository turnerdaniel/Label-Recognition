%% MATLAB Script for the recognition of Expiry Dates from Images
%Reset MATLAB environement
clear; close all; clc;

%TODO:
%Fine-tune Region property Segmentation
%Implement SWT/Gabor/K-Means 
%Potentially Improve post-processing 
%Improve OCR
%Implement date recognition

%% Read image

%imgs = ('img/20 NOV(1184)(2325).jpeg');
%       ('img/25 MAR(2354).jpeg');
%       ('img/10 MAR(1820).jpeg');
%       ('img/image1 2.jpeg');
%       ('img/370 378 988.jpeg');
I = imread('img/988.jpeg');

%% Convert to greyscale
%Check if image is RGB denoted by being 3D array
if size(I,3) > 0
    grey = rgb2gray(I);
else 
    grey = I;
end

%Get dimensions of image
[height, width] = size(grey);

%% Image Pre-processing - Remove Noise, Increase Contrast & Sharpness

%use imtophat to remove uneven illumination?
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
%Laplacian more noisy on some tests
%Sobel/Prewitt inneffective

%Dsiplay pre-processing effects
figure, subplot(2,2,1), imshow(grey), title('Greyscale Image');
subplot(2,2,2), imshow(greyWeiner), title('Linear Weiner Filter');
subplot(2,2,3), imshow(greyContrastStretch), title('Contrast Stretching');
subplot(2,2,4), imshow(greySharp), title('Unsharp Masking');

%% Maximally Stable Extremal Regions (MSER)

%Detect MSER Regions
%RegionAreaRange = region min|max size (30 14000)
%ThresholdDelta = Step size between intensity threshold (2)
%MaxAreaVariation = max area variation between regions (0.25)
mserRegions = detectMSERFeatures(greySharp, 'RegionAreaRange', [150 1500], ...
'ThresholdDelta', 1.5, 'MaxAreaVariation', 0.25);

%Concatenate pixel coordinates as Nx2 matrix
mserPixels = vertcat(cell2mat(mserRegions.PixelList));

%Display MSER Regions overlay on image
figure, imshow(I);
hold on;
plot(mserRegions, 'showPixelList', true, 'showEllipses', false);
title('MSER Regions');
hold off;

%Initialise logical image with necessary dimensions
mserBW = false(height, width);
%Convert img co-ordinates to linear image indexes
ind = sub2ind(size(mserBW), mserPixels(:,2), mserPixels(:,1));
%assign true to co-ordinates that match
mserBW(ind) = true;
figure, imshow(mserBW), title('logical MSER Image');

%% Edge Detection using Canny Edge Detector 
%Canny is good at finding precise edges of text 
edgeBW = edge(greySharp, 'canny');
figure, imshow(edgeBW), title('Canny Edge Image');

%% Image Post-Processing - Opening, 
%Watershed may be good?
%bwmorph for thinning?

seSquare = strel('square', 3);
%seDiamond = strel('diamond', 1);
se = strel('square', 2);
hitmissVert = [0; 1; 0];
hitmissHor = [0 1 0];

%Begin Opening
erode = imerode(mserBW, seSquare);

%Morpholgical Hit or Miss
hm = bwhitmiss(erode, hitmissVert, ~hitmissVert);
hm2 = bwhitmiss(erode, hitmissHor, ~hitmissHor);
outHm = hm | hm2;
hitMiss = erode - outHm;

%Maybe adjust to small se?
dilate = imdilate(hitMiss, seSquare);
%End opening

figure, subplot(2,2,1), imshow(mserBW), title('original');
subplot(2,2,2), imshow(erode), title('Eroded');
subplot(2,2,3), imshow(hitMiss), title('Hit or Miss Performed');
subplot(2,2,4), imshow(dilate), title('Dilated');

%Remove small blobs
clearNoise = bwareaopen(dilate, 100); 
%Close small holes by inverting image between foreground and background
clearSmallHoles = ~bwareaopen(~clearNoise, 3);
figure, imshow(clearSmallHoles), title('No holes & Small Blobs');

%% Remove Unlikely Candidates using Region Properties

% Eccentricity = similar to a line segment (1)
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

%Max euler = -1. However, is affected by noise so change to -3
validEulerNo = [mserStats.EulerNumber] >= -3;
%Remove blobs that are lines (eg. barcodes)
validEccentricity = [mserStats.Eccentricity] < 0.99;
validExtent = [mserStats.Extent] > 0.25 & [mserStats.Extent] < 0.9;
%The ratio between the region and the convex hull
validSolidity = [mserStats.Solidity] > 0.5;
%Make use of vertical and horizontal aspect ratio to ensure shape are
%roughly square = 1
validAspectRatio = aspectRatio < 2.5;

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

% https://cs.adelaide.edu.au/~yaoli/wp-content/publications/icpr12_strokewidth.pdf
% totalObjects = size(mserStats, 1);
% validStrokeWidth = false(1, totalObjects);
% 
% variationThresh = 0.4;%???
% 
% for i = 1:totalObjects
%     
%     cropImage = imcrop(keptObjectsImage, mserStats(i).BoundingBox);
%     regionImage = padarray(cropImage, [1 1]);
%     
%     distanceImage = bwdist(~regionImage);
%     skeletonImage = bwmorph(regionImage, 'thin', inf);
%     
%     strokeWidths = distanceImage(skeletonImage);
%     variation = std(strokeWidths)/mean(strokeWidths);
%     
%     validStrokeWidth(i) = variation > variationThresh;
% end

% helperStrokeWidth() - or could make own... 
% Pseudocode @ Mathworks and journals

%% Gabor Filters/K-Means Clustering/Watershed

% Additional research required...
% See proposal for K-Means method

%% Detected Text

%Display potential text regions
stats = regionprops(keptObjectsImage, 'BoundingBox');
textROI = vertcat(stats.BoundingBox);
textROIImage = insertShape(I, 'Rectangle', textROI, 'LineWidth', 2);
figure, imshow(textROIImage), title('Text ROI');

%1 = calculate bounding boxes and expand by small amount. 
%Overlap = same text region.

%2 = peform morphological operation on text to grow/join letters. Then
%group into bounding boxes

%Maybe grow only horizontally? Dates aren't vertical.
%Rule-based? grow by own size, etc.

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

%% Perform Text Matching using Regex

% May have additional text recognised. eg. descriptions/food title
% eliminate by looking for common data formats:
% DD/MM/YYYY | DD.MM.YYYY | DD MMM or DD JUNE or DD JULY | DD MMMMMMM, etc.
%
% Could also look for dates on y-axis of months of date formats...
% If not found, could repeat ocr with new threshold?

%% Print the Date/Save to File


