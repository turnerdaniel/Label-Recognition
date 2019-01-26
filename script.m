%% MATLAB Script for the recognition of Expiry Dates from Images
%Reset MATLAB environement
clear; close all; clc;

%% Read image & get statistics

%img = imread('img/20 NOV(1184).jpeg');
%img = imread('img/image123.jpeg');
I = imread('img/20 NOV(2325).jpeg');

%% Convert to greyscale
%Check if image is RGB denoted by being 3D array
if size(I,3) > 0
    grey = rgb2gray(I);
else 
    grey = I;
end

[height, width] = size(grey);

%% Increase contrast/clarity?

%use imtophat to remove uneven illumination!

%Perform Contrast Limited Adaptive Histogram Equalisation to help boost
%contrast between text and BG
clahe = adapthisteq(grey);

figure, subplot(1,2,1), imshow(grey), title('Greyscale Image');
subplot(1,2,2), imshow(clahe), title('Contrast Limited Adaptive Hist EQ');

%% Maximally Stable Extremal Regions (MSER)

%Maybe change thresholdDelta(0.8 - 4)?
mserRegions = detectMSERFeatures(clahe, 'RegionAreaRange', [150 1500]);
mserPixels = vertcat(cell2mat(mserRegions.PixelList));

figure, imshow(I);
hold on;
plot(mserRegions, 'showPixelList', true,'showEllipses', false);
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
%letters/numbers found in dates (1234567890abcdefghij_lmnop_rstuv__y_)

%Change 'textLayout' to 'Block'/'line' may help
%Perform morphology to help thin-out connected characters
%Make sure letters are of adequate size (>20px)
