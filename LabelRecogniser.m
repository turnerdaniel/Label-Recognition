classdef LabelRecogniser
    %LabelRecogniser Expiry Date Detection and Recognition Class.
    %
    %Enables the detection and recognition of expiry dates from food 
    %packaging for use with a date verification system by food
    %manufacturers.
    
    properties (Access = public)
        mserSigma = 0.3; % Sigma used for MSER
        % Could put parameters in here (e.g. mser settings, etc.
    end
    properties (Access = private)
        image % Input image
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
            
            if (show == true)
                imshow(img);
            end
        end
    end
    methods (Access = private)
        function grey = convertGrey(obj)
            %convertGrey Convert an RGB image to greyscale
            %   Detailed explanation goes here
            
            if size(obj.image, 3) > 0
                grey = rgb2gray(obj.image);
            else
                grey = img;
            end
        end
    end
end

%TODO:
%Maybe add Image or set this as image class?
%Get and Set functions for the image property - will re-calculate h & w

