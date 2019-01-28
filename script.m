%% MATLAB Script for the recognition of Expiry Dates from Images
%Reset MATLAB environement
clear; close all; clc;

%TODO:
%Contrast Stretching for pre-processing?
%Implement Edge-enhanced MSER
%Fine-tune Region property Segmentation
%Implement SWT/Gabor/K-Means 
%Improve post-processing
%Improve OCR
%Implement date recognition

%% Read image

%imgs = ('img/20 NOV(1184)(2325).jpeg');
%       ('img/25 MAR(2354).jpeg');
%       ('img/image1 2 3.jpeg');
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
%bothat looks good for finding dark text? Maybe not good for light text?

%Perform Median Spatial Filtering to eliminate noise
greyMed = medfilt2(grey, [5 5]);

%Perform Contrast Limited Adaptive Histogram Equalisation (CLAHE)
greyClahe = adapthisteq(greyMed);

%Use unsharp masking to increase image sharpness
greySharp = imsharpen(greyClahe);

%Dsiplay pre-processing effects
figure, subplot(2,2,1), imshow(grey), title('Greyscale Image');
subplot(2,2,2), imshow(greyMed), title('Median Filter');
subplot(2,2,3), imshow(greyClahe), title('CLAHE');
subplot(2,2,4), imshow(greySharp), title('Unsharp Masking');

%% Maximally Stable Extremal Regions (MSER)

%Detect MSER Regions
%RegionAreaRange = region min|max size (30 14000)
%ThresholdDelta = Step size between intensity threshold (2)
%MaxAreaVariation = max area variation between regions (0.25)
mserRegions = detectMSERFeatures(greySharp, 'RegionAreaRange', [150 1500], ...
'ThresholdDelta', 2, 'MaxAreaVariation', 0.25);

%Concatenate pixel coordinates as Nx2 matrix
mserPixels = vertcat(cell2mat(mserRegions.PixelList));

%Display MSER Regions overlay on image
figure, imshow(I);
hold on;
plot(mserRegions, 'showPixelList', true, 'showEllipses', false);
title('MSER Regions');
hold off;

%% Edge Detection using Canny Edge Detector (edge-enhanced MSER)
%Reduce blur from MSER is using edge detection to remove overlapping
%pixels (edge-enhanced MSER)

%------------------------------------
%Currently unsure how to implement correctly!!!
%------------------------------------

%Canny is good at finding precise edges of text 
edgeBW = edge(greySharp, 'canny');

figure, imshow(edgeBW), title('Canny Edge Image');

%Initialise logical image with necessary dimensions
mserBW = false(height, width);
%Convert img co-ordinates to linear image indexes
ind = sub2ind(size(mserBW), mserPixels(:,2), mserPixels(:,1));
%assign 1/true to co-ordinates that match
mserBW(ind) = true;

figure, imshow(mserBW), title('logical MSER Image');

% 
% Need to grow edges along gradient direction to help remove noise?
%
% For the time being, subtract edgeBW to remove noise around letters

%subtractedMserBW = mserBW - edgeBW;

%figure, imshow(subtractedMserBW), title('Subtracted Image');

% Intersection = edgeBW & mserBW; 
% 
% %Use Sobel to get gradient and directions
% [, gAngle] = imgradient(greySharp);
% 
% %Need to Grow Edges using direction of gradient
% 
% %Apparently was a helperGrowEdges function in previous MATLAB. Purpose?
% %grownEdges = helperGrowEdges(Intersection, gAngle, 'LightTextOnDark');
% %figure; imshow(grownEdges); title('Edges grown along gradient direction')

%% Image Post-Processing - Opening, 
%more experimentation needed

seSquare = strel('square', 3);
%seDisk = strel('disk', 3);

openMserBW = imopen(mserBW, seSquare);

figure, imshow(openMserBW), title('Opening Performed');

%% Remove Unlikely Candidates using Region Properties

% Eccentricity = similar to a line segment (1)
% Euler Number = Don't have many holes (max 2 | B)
% Aspect Ratio = mostly square
% Extent = have very high or very low occupation of bounding box (O vs l)
% bwareaopen = letters are too big/too small
% Touching the border

mserLabel = bwlabel(openMserBW);
mserStats = regionprops(openMserBW, 'Area','Eccentricity', 'EulerNumber', 'Extent');

%Max euler number = -1. However, is affected by noise so change to -3

% valid_ = [mserStats._] > 0;

% %Find the index of all objects that have valid properties
% keptObjects = find(valid_);
% %Return pixels that satisfy the similar property criteria for the image
% keptObjectsImage = ismember(mserLabel, keptObjects);

areaMserBW = bwareaopen(openMserBW, 150);

figure, imshow(areaMserBW), title('Filter images using text properties')

%% Stroke Width Transform

% helperStrokeWidth() - or could make own... 
% Pseudocode @ Mathworks and journals

%% Gabor Filters/K-Means Clustering

%Additional research required...

%% Detected Text

%Display potential text regions
stats = regionprops(areaMserBW, 'BoundingBox');
textROI = vertcat(stats.BoundingBox);
textROIImage = insertShape(I, 'Rectangle', textROI, 'LineWidth', 2);
figure, imshow(textROIImage), title('Text ROI');

%1 = calculate bounding boxes and expand by small amount. 
%Overlap = same text region.

%2 = peform morphological operation on text to grow/join letters. Then
%group into bounding boxes

%% Perform Optical Character Recognition (OCR)

detectedText = ocr(I);
[detectedText.Text]

%improve by creating a ocr characterSet to limit the possible characters to
%letters/numbers found in dates (1234567890 abcdefghij_lmnop_rstuv__y_ /.)

%Change 'textLayout' to 'Block'/'line' may help
%Perform morphology to help thin-out connected characters
%Make sure letters are of adequate size (>20px)
%Use ROI for OCR

%% Perform Text Matching using Regex

% May have additional text recognised. eg. descriptions/food title
% eliminate by looking for common data formats:
% DD/MM/YYYY | DD.MM.YYYY | DD MMM | DD MMMMMMM, etc.
