function alterFilePaths(gTruth, newPath)
%ALTERFILEPATHS Change file paths of a groundTruth object.
%This is an alternative function to changeFilePaths() introduced in MATLAB 
%2018b which is not installed on the University of Lincoln computers. 
%
% Syntax: alterFilePaths(gTruth, newPath)
%
% Inputs:
%   gTruth  - Handle of groundTruth object that needs file paths changed.
%             This is passed by reference.
%   newPath - Filepath to new directory holding the dataset.
%
% See also: CHANGEFILEPATHS

%Allocate variables to hold all filepaths
length = size(gTruth.DataSource, 1);
images = cell(length, 1);

for i = 1:length
    %Get filenames of images
    [~, name, ext] = fileparts(gTruth.DataSource{i});
    %Append filenames to new path
    images{i} = fullfile(newPath, strcat(name, ext));
end

%Create new image source for groundTruth from new paths
imageSource = groundTruthDataSource(images);

%Alter groundTruth handle object with rectified path
gTruth.DataSource = imageSource;