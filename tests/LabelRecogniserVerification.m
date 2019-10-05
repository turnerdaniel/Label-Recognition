classdef LabelRecogniserVerification
    %LabelRecogniserVerification Verify the performance of LabelRecogniser
    %algorithm.
    %   
    %   Collect metrics from LabelRecogniser.recogniseDates() that measure
    %   date detection and recognition accuracy. These include:
    %
    %       Precision, Recall, Recognition Acuracy, Overall Accuracy, 
    %       Minimum Duration, Max Duration, Mean Duration, Standard
    %       Deviation Duration and Standard Error of the Mean Duration.
    %
    %   LabelRecogniserVerification verifies the performance of the
    %   LabelReocgniser algorithm on the entire dataset (500 images).
    %
    %   LabelRecogniserVerification(N) verifies the performance of the
    %   LabelRecogniser algorithm on N randomly-selected images from the
    %   dataset.
    
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
        groundTruth % Table holding bounding box position and dates for the dataset
        precisions % Vector of precision values
        recalls % Vector of recall values
        matches % Vector of matching detected and ground truth values
        durations % Vector of algorithm durations
        noImages % Number of image in dataset
    end
    
    methods
        function obj = LabelRecogniserVerification()
            %LabelRecogniserVerification Construct and run verification of the algorithm on the dataset
            
            %Load ground truth data 
            obj.groundTruth = load('labels/groundTruth.mat');
            
            %Pre-allocate vectors
            obj.noImages = size(obj.groundTruth, 1);
            obj.precisions = zeros(obj.noImages, 1);
            obj.recalls = zeros(obj.noImages, 1);
            obj.matches = false(obj.noImages, 1);
            obj.durations = zeros(obj.noImages, 1);
            
            %Generate metrics
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
                %Initialise LabelRecogniser with dataset image
                lr = LabelRecogniser(obj.groundTruth.Source{i});
                
                %Measure execution time and perform recognition
                tic;
                [dates, bbox] = lr.recogniseDates();
                obj.durations(i) = toc;
                
                %Calculate precision and recall from bounding boxes
                [obj.precisions(i), obj.recalls(i)] = ...
                    bboxPrecisionRecall(bbox, obj.groundTruth.Position{i});
                
                %Check that a date was a recognised
                if isempty(dates)
                    %skip iteration
                    continue
                else
                    %Remove spaces
                    nsKnownDate = strrep(obj.groundTruth.Date{i}, ' ', '');
                    %Loop through 1 or more recognised dates
                    for j = 1:size(dates, 1)
                        %Remove spaces
                        nsDate = strrep(dates(j), ' ', '');
                        %Check that a match has not already been found
                        if obj.matches(i) == false
                            %Case-insensitive comparison of date and ground truth match
                            obj.matches(i) = strcmpi(nsDate, nsKnownDate);
                        else
                            %Exit loop as match already found
                            break
                        end
                    end
                end
            end
            
            %Calculate metrics from vectors and assign to properties
            obj.Precision = mean(obj.precisions);
            obj.Recall = mean(obj.recalls);
            
            %Create logical index of all detected dates to remove failed
            %detections from recognition accuracy calculation
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