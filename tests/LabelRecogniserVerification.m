classdef LabelRecogniserVerification
    %LabelRecogniserVerification Verify the accuracy of LabelRecogniser algorithm
    %   
    %   Collect metrics from LabelRecogniser.recogniseDates() that measure
    %   date detection and recognition accuracy.
    
    %TODO:
    %   Mention not detected but recognised...
    %   It is possible for dates to be incorrectly detected but recognise
    %   at the end ue to threshold necessary for a bounding box to be
    %   considered accurate.
    
    
    properties
        Precision % How relevant the bounding boxes were [0 1]
        Recall % How many relevant bounding boxes were returned [0 1]
        RecognitionAccuracy % How many dates were recognised correctly from successful detections [0 1]
        OverallAccuracy % How many dates were correctly recognised out of the dataset [0 1]
        MinDuration % The quickest time that recogniseDates() was completed on the dataset
        MaxDuration % The slowest time that recogniseDates() was completed on the dataset
        MeanDuration % The average time that recogniseDates() took to complete
        StdDevDuration % The variation in completion times of recogniseDates() across the dataset
        StdErrorDuration % How well MeanDuration represents the completion times of recogniseDates() outside of the dataset (population) 
    end
    
    properties (Access = private)
        groundTruth %
        precisions %
        recalls %
        matches %
        durations %
        noImages %
    end
    
    methods
        function obj = LabelRecogniserVerification()
            obj.groundTruth = load('labels/groundTruth.mat');
            
            obj.noImages = size(obj.groundTruth, 1);
            obj.precisions = zeros(obj.noImages, 1);
            obj.recalls = zeros(obj.noImages, 1);
            obj.matches = false(obj.noImages, 1);
            obj.durations = zeros(obj.noImages, 1);
            
            obj = obj.getMetrics();
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
        function obj = getMetrics(obj)
            %Add LabelRecogniser to path
            addpath('..');
            %Remove LabelRecogniser from path when destroyed (Anon function)
            cleanupObj = onCleanup(@() rmpath('..'));
            
            for i = 1:obj.noImages
                lr = LabelRecogniser(obj.groundTruth.Source{i});
                
                tic;
                [dates, bbox] = lr.recogniseDates();
                obj.durations(i) = toc;
                
                [obj.precisions(i), obj.recalls(i)] = ...
                    bboxPrecisionRecall(bbox, obj.groundTruth.Position{i});
                
                % --------------------------------------------------------
                %TODO:
                    %Use image set instead of redefining lr
                    
                if isempty(dates)
                    continue
                else
                    nsKnownDate = strrep(obj.groundTruth.Date{i}, ' ', '');
                    
                    for j = 1:size(dates, 1)
                        nsDate = strrep(dates(j), ' ', '');
                        
                        if obj.matches(i) == false
                            obj.matches(i) = strcmpi(nsDate, nsKnownDate);
                        else
                            break
                        end
                    end
                end
            end
            
            %Calculations
            obj.Precision = mean(obj.precisions);
            obj.Recall = mean(obj.recalls);
            
            detected = (obj.recalls == 1);
            obj.RecognitionAccuracy = sum(obj.matches(detected)) / sum(detected);
            obj.OverallAccuracy = sum(obj.matches) / obj.noImages;
            
            obj.MeanDuration = mean(obj.durations);
            obj.MinDuration = min(obj.durations);
            obj.MaxDuration = max(obj.durations);
            obj.StdDevDuration = std(obj.durations);
            obj.StdErrorDuration = obj.StdDevDuration / sqrt(obj.noImages);          
        end
        
    end
end

