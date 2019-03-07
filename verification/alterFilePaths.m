function gt = alterFilePaths(gTruth, newPath)
%ALTERFILEPATHS changes the file paths for a groundTruth object

%supply:
%gTruth object
%filepath to dataset folder ending with '/' or '\'

length = size(gTruth.DataSource, 1);
images = cell(length, 1);

for i = 1:length
    [~, name, ext] = fileparts(gTruth.DataSource{i});
    images{i} = fullfile(newPath, strcat(name, ext));
end

imageSource = groundTruthDataSource(images);

gt = groundTruth(imageSource, gTruth.LabelDefinitions, gTruth.LabelData);