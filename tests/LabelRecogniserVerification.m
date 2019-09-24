classdef LabelRecogniserVerification
    %LabelRecogniserVerification Verify the accuracy of LabelRecogniser algorithm
    %   
    %   Collect metrics from LabelRecogniser.recogniseDates() that measure
    %   date detection and recognition accuracy.
    
    %TODO: 
    % maybe result class?
    
    properties
        Precision %
        Recall %
        RecognitionAccuracy %
        OverallAccuracy %
        MinDuration %
        MaxDuration %
        MeanDuration %
        StdDevDuration %
        StdErrorDuration %
    end
    
    properties (Access = private)
        groundTruth %
        precisions
        recalls
        times
        noImages %
    end
    
    methods
        function obj = LabelRecogniserVerification()
            obj.groundTruth = load('labels/groundTruth.mat');
            
            obj.noImages = size(obj.groundTruth, 1);
            obj.precisions = zeros(obj.noImages, 1);
            obj.recalls = zeros(obj.noImages, 1);
            obj.times = zeros(obj.noImages, 1);
            
            obj.getMetrics();
        end
        
        function obj = set.groundTruth(obj, value)
            %Determine data type of value
            switch class(value)
                case 'struct'
                    %See if struct contains the ground truth
                    try 
                        gt = value.gt;
                    catch
                        error("LabelRecogniserVerification:InvalidStruct", ...
                            "Ground Truth not found in struct. Ensure it is named 'gt.'");
                    end
                    %Check that the ground truth is a table
                    if istable(gt)
                        %Assign corrected table to groundTruth
                        obj.groundTruth = correctPaths(gt);
                    else
                        error("LabelRecogniserVerification:InvalidStructDataType", ...
                            "The struct must contain a table.");
                    end
                    
                case 'table'
                    %Assign corrected table to groundTruth
                    obj.groundTruth = correctPaths(value);
                    
                otherwise
                    error("LabelRecogniserVerification:InvalidDataType", ...
                        "Value must be a table or a struct containing a table.");
            end
            
            %Local function that converts relative paths to absolute paths
            %within ground truth table
            function groundTruth = correctPaths(groundTruth)
                length = size(groundTruth, 1); %Get the number of elements to loop through
                currentDir = pwd; %Get the current path
                targetDir = fileparts(groundTruth.Source{1}); %Get the name of the relative directory
                
                %Check to see if relative target directory exists
                if isfolder(targetDir)
                    %Loop through table
                    for i = 1:length
                        %Get filenames of images
                        relativePath = groundTruth.Source{i};
                        %Construct absolute filepath
                        absolutePath = fullfile(currentDir, relativePath);
                        %Save to table
                        groundTruth.Source{i} = absolutePath;
                    end
                else
                    %Throw error for directory not existing
                    error("LabelRecogniserVerification:InvalidPath", ...
                        "The relative path does not exist.");
                end
            end
        end
    end
    
    methods (Access = private)
        function result = getMetrics(obj)
            
            for i = 1:obj.noImages
                %lr = LabelRecogniser(obj.groundTruth.Source{i});
                %[text, bbox, ...] = lr.recogniseDates();
            end
            
            result = obj.groundTruth;
        end
    end
end

