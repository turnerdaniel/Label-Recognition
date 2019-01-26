%% MATLAB Script for the recognition of Expiry Dates from Images
%Reset MATLAB environement
clear; close all; clc;

%% Read image & get statistics

%img = imread('img/20 NOV(1184).jpeg');
%img = imread('img/image123.jpeg');
I = imread('img/20 NOV(2325).jpeg');

[height, width] = size(I);

%% Convert to greyscale
%Check if image is RGB denoted by being 3D array
if size(I,3) > 0
    grey = rgb2gray(I);
else 
    grey = I;
end

%% Increase contrast/clarity?

clahe = adapthisteq(grey);

figure, subplot(1,2,1), imshow(grey), title('greyscale image');
subplot(1,2,2), imshow(clahe), title('Contrast Limited Adaptive Hist EQ');

%% Maximally Stable Extremal Regions (MSER)

mserRegions = detectMSERFeatures(clahe,'RegionAreaRange',[150 1500]);
mserPixels = vertcat(cell2mat(mserRegions.PixelList));

figure, imshow(I);
hold on;
plot(mserRegions, 'showPixelList', true,'showEllipses', false);
title('MSER:');
hold off;

%Convert MSER pixel lists to a binary mask
% mserBW = false(size(grey));
% ind = sub2ind(size(mserMask), mserPixels(:,2), mserPixels(:,1));
% mserMask(ind) = true;

%% Edge Detection using Canny Edge Detector

%Canny was much better at finding the edge of text 
edgeMask = edge(grey, 'canny');

figure, imshow(edgeMask), title('Edge Image');

% Find intersection between edges and MSER regions
% Intersection = edgeMask & mserMask; 
% figure; imshowpair(edgeMask, Intersection, 'montage'); 
% title('Canny edges and intersection of canny edges with MSER regions')



%% Perform Optical Character Recognition (OCR)

ocrtxt = ocr(I);
[ocrtxt.Text]