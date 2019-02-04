%% MATLAB Script for the MSER Testing
%Reset MATLAB environement
clear; clc;


%% Read image

%imgs = ('img/20 NOV(1184)(2325).jpeg');
%       ('img/25 MAR(2354).jpeg');
%       ('img/10 MAR(1820).jpeg');
%       ('img/image1 2 3 4.jpeg');
%       ('img/370 378 988.jpeg');

%I = imread('img/image1.jpeg');
%I = imread('img/988.jpeg');
I = imread('img/20 NOV(1184).jpeg');

Delta = 2.5;
Var = 0.2;

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

% figure, subplot(2,2,1), imshow(grey), title('Greyscale Image');
% subplot(2,2,2), imshow(greyWeiner), title('Linear Weiner Filter');
% subplot(2,2,3), imshow(greyContrastStretch), title('Contrast Stretching');
% subplot(2,2,4), imshow(greySharp), title('Unsharp Masking');

%% Maximally Stable Extremal Regions (MSER)

%Detect MSER Regions
%RegionAreaRange = region min|max size (30 14000)
%ThresholdDelta = Step size between intensity threshold (2)
%MaxAreaVariation = max area variation between regions (0.25)
mserRegions = detectMSERFeatures(greySharp, 'RegionAreaRange', [150 1500], ...
'ThresholdDelta', Delta, 'MaxAreaVariation', Var);

%Display MSER Regions overlay on image
figure, imshow(I);
hold on;
plot(mserRegions, 'showPixelList', true, 'showEllipses', false);
title(['MSER Regions Delta = ', num2str(Delta), ' Variance = ', num2str(Var)]);
hold off;