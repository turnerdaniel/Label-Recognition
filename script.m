%% MATLAB Script for the recognition of Expiry Dates from Images
%Reset MATLAB environement
clear; close all; clc;

%% Read image

%imgs = ('img/20 NOV(1184)(2325).jpeg');
%       ('img/25 MAR(2354).jpeg');
%       ('img/image1 2 3.jpeg');
%       ('img/370 378 988.jpeg');
I = imread('img/image2.jpeg');

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
%RegionAreaRange = region min|max size
%ThresholdDelta = Step size between intensity threshold
%MaxAreaVariation = max area variation between regions
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

%% Edge Detection using Canny Edge Detector (edge-enhanced MSER)%
%
% %Reduce blur from MSER is using edge detection to remove overlapping
% %pixels (edge-enhanced MSER)
% 
% %Canny was much better at finding the edge of text 
% edgeMask = edge(clahe, 'canny');
% 
% figure, imshow(edgeMask), title('Edge Image');
% 
% %Convert MSER pixels to binary image
% mserBW = false(height, width);
% %Convert img coords to linear image index
% ind = sub2ind(size(mserBW), mserPixels(:,2), mserPixels(:,1));
% mserBW(ind) = true;
% 
% Intersection = edgeMask & mserBW; 
% 
% %Use Sobel to get gradien and directions
% [, gTheta] = imgradient(clahe);
% 
% %Need to Grow Edges using direction of gradient
% 
% %You must specify if the text is light on dark background or vice versa
% %gradientGrownEdgesMask = helperGrowEdges(Intersection, gTheta, 'LightTextOnDark');
% %figure; imshow(gradientGrownEdgesMask); title('Edges grown along gradient direction')

%% Remove Unlikely Candidates using Region Properties

% Eccentricity = similar to a line segment (1)
% Euler Number = Don't have many holes (max 2 | B)
% Aspect Ratio = mostly square
% Extent = have very high or very low occupation of bounding box (O vs l)

%bwareaopen = letters arent too big/too small

%% Stroke Width Transform

% helperStrokeWidth() - or could make own:
% https://github.com/tanmayGIT/ICDAR-2015-DTW/blob/master/helperStrokeWidth.m

%% Region Growing based on colour?

% text will always be black
% Hard to find start condition

%% Connected text

%1 = calculate bounding boxes and expand by small amount. Overlap is the
%same text region.

%2 = peform morphological operation on text to grow/join letters. Then
%group into bounding boxes

%% Perform Optical Character Recognition (OCR)

ocrtxt = ocr(I);
[ocrtxt.Text]

%improve by creating a ocr characterSet to limit the possible characters to
%letters/numbers found in dates (1234567890 abcdefghij_lmnop_rstuv__y_ /.)

%Change 'textLayout' to 'Block'/'line' may help
%Perform morphology to help thin-out connected characters
%Make sure letters are of adequate size (>20px)
%Increase size of bbox/image

