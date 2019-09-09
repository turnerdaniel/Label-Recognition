%% MATLAB Script for the recognition of Food Expiry Dates from Images
%Author: Daniel Turner
%University: University of Lincoln
%Date: 09/09/2019

%% Setup Script Environment

%Reset MATLAB environment
clear; close all; clc;

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

%% Perform Recognition

lr = LabelRecogniser(I);
dates = lr.recogniseDates();

%% Print the Dates or Save to File

%Check to see if dates have been detected
if (size(dates, 1) > 0)
    %create message holding the detected dates
    data = dates;
    caption = 'Success';
    message = (['Managed to find the following expiry dates:'; data]);
    
    %Create message box that displays message with option to save to file
    buttonPress = questdlg(message, caption, 'Okay', 'Save...', 'Okay');
else
    %create error message
    caption = 'Error!';
    message = ('Couldn''t find any expiry dates');
    
    %Create message box that displays message 
    buttonPress = questdlg(message, caption, 'Okay', 'Okay');
end

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

