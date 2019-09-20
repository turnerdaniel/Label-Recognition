function gtTable = convertPaths(gtTable)
%convertPaths Convert relative path names to absolute ones within a table.
%
%   gtTable = convertPaths(gtTable) performs relative to absolute path 
%   conversion on gtTable and outputs the adjusted table.
%
%   convertPaths was custom built for converting groundTruth data for the
%   LabelRecogniser class. Functionality outside of this projet is not
%   guranteed.
%
%   See also changeFilePaths.

    %Check that input argument is a table
    if ~istable(gtTable)
        error("Input argument must be a table.");
    end
        
    length = size(gtTable, 1); %Get the number of elements to loop through
    currentDir = pwd; %Get the current path
    targetDir = fileparts(gtTable.Source{1}); %Get the name of the relative directory
    
    %Check to see if target relative directory exists
    if isfolder(targetDir)
        %Loop through table
        for i = 1:length
            %Get filenames of images
            relativePath = gtTable.Source{i};
            %Construct absolute filepath
            absolutePath = fullfile(currentDir, relativePath);
            %Save to table
            gtTable.Source{i} = absolutePath;
        end
    else
        error("The relative path does not exist.");
    end
end

