classdef LabelRecogniser
    %LabelRecogniser Expiry Date Detection and Recognition Class.
    %
    %Enables the detection and recognition of expiry dates from food 
    %packaging for use with a date verification system by food
    %manufacturers.
    
    properties 
        image; % Image used for recognition
        thresholdDelta = 0.3; % Delta used for MSER
    end
    properties (Access = private)
        h % height of input image
        w % width of input image
    end
    
    methods
        function obj = LabelRecogniser(imageFilePath)
            %LabelRecogniser Construct an instance of this class from a filepath to an image
            %
            %Must be in a valid image format (.jpeg, .png, etc.)
            try
                obj.image = imread(imageFilePath);
                [obj.h, obj.w, ~] = size(obj.image);
            catch
                error("LabelRecogniser:BadFile", "Cannot read image due to an invalid file path or type.");
            end
        end
        
        function recogniseDates(obj, show)
            %recogniseDates Identify the position and textual representation of the dates shown within the image.
            grey = convertGrey(obj, obj.image);
            img = preProcess(obj, grey);
            img = mser(obj, img);
            img = postProcess(obj, img);
            img = connectedComponentEnhance(obj, img, grey);
            img = geometricFilter(obj, img);
            
            if show == true
                figure, imshow(img), title("img");
            end
        end
    end
    methods (Access = private)
        function out = convertGrey(~, image)
            %convertGrey Convert an RGB image to greyscale
            
            if size(image, 3) > 0
                out = rgb2gray(image);
            else
                out = image;
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
        
        function out = postProcess(~, image)
            %postProcess Cleanup MSER binary image
            
            %Define 3x3 square structuring element
            element = strel('square', 3);
            
            %Perform opening to remove protrusions and small joins
            open = imopen(image, element);
            %Remove blobs with an area less than 100 px
            clearNoise = bwareaopen(open, 100);
            %Fill small holes
            out = ~bwareaopen(~clearNoise, 3);
        end
        
        function out = connectedComponentEnhance(obj, image, greyscale)
            %connectComponentEnhance
            
            %Something wrong here...
            
            stats = regionprops(image, 'Image', 'BoundingBox');
            numObjects = size(stats, 1);
            adjusted = false(obj.h, obj.w);
            
            for i = 1:numObjects
                %Just floor? bbox = floor(stats(i).BoundingBox);
                bbox = ceil(stats(i).BoundingBox - [0, 0, 1, 1]);
                imageSegment = stats(i).Image;
                
                binarySegment = imbinarize(imcrop(greyscale, bbox));
                %maybe 2 outputs for pre-process
                
                ccRegularImage = bwconncomp(imageSegment & binarySegment);
                ccInverseImage = bwconncomp(imageSegment & ~binarySegment);
                
                if ccRegularImage.NumObjects == 0 && ccInverseImage.NumObjects ~= 0
                    %Use inverse image
                    keepImage = imageSegment & ~binarySegment;
                elseif ccInverseImage.NumObjects == 0 && ccRegularImage.NumObjects ~= 0
                    %Use regular image
                    keepImage = imageSegment & binarySegment;
                elseif ccRegularImage.NumObjects < ccInverseImage.NumObjects
                    %Use regular image
                    keepImage = imageSegment & binarySegment;
                elseif ccInverseImage.NumObjects < ccRegularImage.NumObjects
                    %Use inverse image
                    keepImage = imageSegment & ~binarySegment;
                %Edge case where they could have matching number of objects
                elseif ccInverseImage.NumObjects == ccRegularImage.NumObjects
                    %Concatenate list to handle multiple objects in an image
                    ccRegularSize = size(cat(1, ccRegularImage.PixelIdxList{:}), 1);
                    ccInverseSize = size(cat(1, ccInverseImage.PixelIdxList{:}), 1);
                    %Choose threshold that produces the most pixels
                    if ccRegularSize < ccInverseSize 
                        keepImage = imageSegment & ~binarySegment;
                    else
                        keepImage = imageSegment & binarySegment;
                    end
                else
                    %Use regular image as last resort
                    keepImage = imageSegment & binarySegment;
                end
                
                %Replace existing image locations with new enhanced images
                adjusted(bbox(2):bbox(2) + bbox(4), bbox(1):bbox(1) ...
                    + bbox(3)) = keepImage;  
            end
            
            %Remove blobs with an area below 100 pixels
            clearNoise = bwareaopen(adjusted, 20); 
            %Fill small holes by inverting image so the background becomes foreground
            out = ~bwareaopen(~clearNoise, 3);
        end
        
        function out = geometricFilter(~, image)
            
            %Label the image and get the properties of each object
            label = bwlabel(image);
            stats = regionprops(image, 'BoundingBox', 'Eccentricity', ...
                'EulerNumber', 'Extent', 'Solidity');

            %Calculte the maximum aspect ratio for horizontal and vertical direction
            bboxes = vertcat(stats.BoundingBox);
            bbWidths = bboxes(:, 3).';
            bbHeights = bboxes(:, 4).';
            aspectRatio = max(bbWidths ./ bbHeights, bbHeights ./ bbWidths);

            %Keep blobs that have less than 3 holes in them. Need to compensate for noise
            validEulerNo = [stats.EulerNumber] >= -2;
            %Keep blobs that are not perfect lines (eg. barcodes)
            validEccentricity = [stats.Eccentricity] < 0.99;
            %Keep blobs that fall within normal distribution of Area to BBox ratio
            validExtent = [stats.Extent] > 0.25 & [stats.Extent] < 0.9;
            %Keep blobs that do not have significant convex areas
            validSolidity = [stats.Solidity] > 0.4;
            %Keep blobs that are not extremely elongated 
            validAspectRatio = aspectRatio < 3;

            %Find the index of all objects that have valid properties
            keep = find(validEulerNo & validEccentricity & validExtent & ...
                validSolidity & validAspectRatio);
            %Return pixels that are at the valid indexes for the image
            out = ismember(label, keep);
        end
    end
end

%TODO:
%Maybe outputs should have actual names (not out?)
%Get and Set functions for the image property - will re-calculate h & w
    %Dependant variable?

