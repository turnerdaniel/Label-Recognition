classdef LabelRecogniser
    %LabelRecogniser Expiry Date Detection and Recognition Class.
    %
    %Enables the detection and recognition of expiry dates from food 
    %packaging for use with a date verification system by food
    %manufacturers.
    
    properties (Access = public)
        image % Image used for recognition
        thresholdDelta = 0.3; % Delta used for MSER
    end
    properties (Access = private)
        h % height of input image
        w % width of input image
    end
    
    methods (Access = public)
        function obj = LabelRecogniser(imageFilePath)
            %LabelRecogniser Construct an instance of this class from a filepath to an image
            %
            %Must be in a valid image format (.jpeg, .png, etc.)
            try
                obj.image = imread(imageFilePath);
                [obj.h, obj.w, ~] = size(obj.image);
            catch
                error("Cannot read image due to an invalid file path or type.");
            end
        end
        
        function recogniseDates(obj, show)
            %recogniseDates Identify the position and textual representation of the dates shown within the image.
            img = convertGrey(obj);
            img = preProcess(obj, img);
            img = mser(obj, img);
            
            if (show == true)
                imshow(img);
            end
        end
    end
    methods (Access = private)
        function out = convertGrey(obj)
            %convertGrey Convert an RGB image to greyscale
            
            %Maybe use image like other functions?
            if size(obj.image, 3) > 0
                out = rgb2gray(obj.image);
            else
                out = obj.image;
            end
        end
        
        function out = preProcess(~, image)
            %preProcess Perform image enhancements to improve text clarity
            
            %Perform Spatial Filtering to eliminate noise
            noise = wiener2(image, [3 3]);
            %Perform Linear Contrast Stretching
            contrast = imadjust(noise);
            %Use Unsharp Masking to increase image sharpness
            out = imsharpen(contrast);
        end
        
        function out = mser(obj, image)
            %mser Detect text candiates using Maximally Stable Extremal Regions
            
            %Initialise logical image with necessary dimensions
            out = false(obj.h, obj.w);
            
            %Could make parameters editable?
            mser = detectMSERFeatures(image, 'RegionAreaRange', [150 1500], ...
                'ThresholdDelta', 2.5, 'MaxAreaVariation', 0.2);
            
            %Concatenate pixel coordinates from MSER as Nx2 matrix
            pixels = cell2mat(mser.PixelList);            
            %Convert img co-ordinates to linear image indexes
            ind = sub2ind([obj.h, obj.w], pixels(:,2), pixels(:,1));
            %Set matching pixels to white
            out(ind) = true;
        end
    end
end

%TODO:
%Maybe add Image or set this as image class?
%Get and Set functions for the image property - will re-calculate h & w

