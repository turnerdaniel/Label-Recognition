classdef LabelRecogniser
    %LabelRecogniser Expiry Date Detection and Recognition Class.
    %
    %Enables the detection and recognition of expiry dates from food 
    %packaging for use with a date verification system by food
    %manufacturers.
    
    properties 
        image; % Image used for recognition
        thresholdDelta = 0.3; % Delta used for MSER
        strokeWidthVariationThreshold = 0.4; % Maximum difference in the width of each stroke in letters
    end
    properties (Access = private)
        h % height of input image
        w % width of input image
    end
    
    methods
        function obj = LabelRecogniser(imageFilePath)
            %LabelRecogniser Construct an instance of this class from a filepath to an image
            %
            obj.image = imageFilePath;
            
            [obj.h, obj.w, ~] = size(obj.image); %dependant vars
        end
        
        function recogniseDates(obj, show)
            %recogniseDates Identify the position and textual representation of the dates shown within the image.
            img = convertGrey(obj, obj.image);
            grey = preProcess(obj, img);
            img = mser(obj, grey);
            img = postProcess(obj, img);
            img = connectedComponentEnhance(obj, img, grey);
            img = geometricFilter(obj, img);
            img = strokeWidthTransform(obj, img);
            bboxes = textGrouping(obj, img);
            
            if show == true
                figure, imshow(img), title("img");
            end
        end
        
        function obj = set.image(obj, value)
            disp('setter called')
            switch class(value)
                case {'char', 'string'}
                    try
                        obj.image = imread(value);
                    catch
                        error("LabelRecogniser:BadImageFile", "Cannot read image due to an invalid file type.");
                    end
                    
                case 'uint8'
                    if ~isvector(value)
                        obj.image = value;
                    else
                        error("LabelRecogniser:BadImageDimensions", "Image dimensions are too small.");
                    end
                    
                otherwise
                    error("LabelRecogniser:BadImage", "Invalid data type. Must be a file location or uint8 array.");
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
            %connectComponentEnhance Perform connected component enhancement to clean up binary MSER image
            
            stats = regionprops(image, 'Image', 'BoundingBox');
            numObjects = size(stats, 1);
            adjusted = false(obj.h, obj.w);
            
            for i = 1:numObjects
                %%%%%Just floor? bbox = floor(stats(i).BoundingBox);
                %Get bounding box and image for current object
                bbox = ceil(stats(i).BoundingBox - [0, 0, 1, 1]);
                imageSegment = stats(i).Image;
                %Threshold the cropped greyscale image using Otsu's Method
                binarySegment = imbinarize(imcrop(greyscale, bbox));
                %Identify connected components in the intersection between binary image
                %and thresholded image
                ccRegularImage = bwconncomp(imageSegment & binarySegment);
                ccInverseImage = bwconncomp(imageSegment & ~binarySegment);
                
                %Check objects to find which has the fewest number of components above 0 which
                %will likely have the appropriate threshold
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

            %Calculate the maximum aspect ratio for horizontal and vertical direction
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
        
        function out = strokeWidthTransform(~, image)
            %Label the image and get the smallet image encapsulating each object
            label = bwlabel(image);
            stats = regionprops(image, 'Image');

            %Initialise variables that hold the number of objects and their stroke
            %width variations
            numObjects = size(stats, 1);
            swVariations = zeros(1, numObjects);

            %Define the maximum variation in stroke width for letters
            swVariationThresh = 0.4;

            %Loop through all objects in image
            for i = 1:numObjects
                %Get the image conting just the object
                imageSegment = stats(i).Image;

                %Pad the image with 0's to avoid stroke width being affected by boundary
                paddedimage = padarray(imageSegment, [1 1]);

                %Perform Distance Transform
                distanceTransform = bwdist(~paddedimage);

                %Perform thinning until Skeleton is created
                skeletonTransform = bwmorph(paddedimage, 'thin', inf);

                %Retrieve the stroke width values for the image
                strokeWidths = distanceTransform(skeletonTransform);

                %Calculate the variation in stroke widths
                swVariations(i) = std(strokeWidths)/mean(strokeWidths);
            end

            %Find valid stroke widths that are below the variation threshold
            validStrokeWidths = swVariations < swVariationThresh;
            %Find the index of these objects
            keep = find(validStrokeWidths);
            %Create an image made of objects that are below the variation threshold
            out = ismember(label, keep);
        end
        
        function out = textGrouping(obj, image)
            %Get the bounding box for each object and convert to usable coordinates
            textStats = regionprops(image, 'BoundingBox');
            textROI = vertcat(textStats.BoundingBox);

            %Seperate bounding box into seperate variables for manipulation
            roiX = textROI(:, 1);
            roiY = textROI(:, 2);
            roiW = textROI(:, 3);
            roiH = textROI(:, 4);

            %Expand ROI by 2/3 the character width in horizontal direction 
            expandedX = roiX - (roiW * (2/3)); 
            expandedW = roiW + ((roiW * (2/3)) * 2);

            %Ensure that ROI is within bounds of the image
            expandedX = max(expandedX, 1);
            expandedW = min(expandedW, obj.w - expandedX);

            %Create expanded bounding boxes
            expandedTextROI = [expandedX, roiY, expandedW, roiH];
            
            %Calculate the ratio of union between bounding boxes
            overlapRatio = bboxOverlapRatio(expandedTextROI, expandedTextROI, 'Union');
            overlapSize = size(overlapRatio, 1);

            %Remove union with own Bounding Box
            overlapRatio(1:overlapSize + 1:overlapSize^2) = 0;

            %Create node graph of connected Bounding Boxes & find the index of boxes
            %that intersect
            labelledROI = conncomp(graph(overlapRatio));

            %Find the total number of bounding boxes
            totalComponents = max(labelledROI);

            %loop through connected bounding boxes
            for i = 1:totalComponents
                %find the index of connected bounding boxes
                connectedBoxes = find(labelledROI == i);
                %Get their heights
                heightOfBoxes = roiH(connectedBoxes);

                %Calculate the average height of connected bounding boxes
                meanHeight = mean(heightOfBoxes);
                meanError = meanHeight/2;

                %Find all bounding boxes that have a height that matches the criteria
                validBoxes = heightOfBoxes > meanHeight - meanError & heightOfBoxes < meanHeight + meanError;
                %Get the index of all bounding boxes that don't match the criteria
                invalidHeight = connectedBoxes(validBoxes == 0);

                %Check that invalidHeights is not empty
                if (~isempty(invalidHeight))
                    %Loop through invalid indexes
                    for j = 1:size(invalidHeight, 2)
                        %Get the id of the bounding box with an invalid height
                        id = invalidHeight(j);
                        %Seperate the component into a new indices by creating
                        %a new 'label' above max value
                        labelledROI(id) = max(labelledROI) + 1;
                    end
                end
            end

            %Find the minimum values of X & Y and the maximum values of W & H for each 
            %of the intersecting bounding boxes to form encompassing bounding boxes
            labelledROI = labelledROI.';
            x1 = accumarray(labelledROI, expandedX, [], @min);
            y1 = accumarray(labelledROI, roiY, [], @min);
            x2 = accumarray(labelledROI, expandedX + expandedW, [], @max);
            y2 = accumarray(labelledROI, roiY + roiH, [], @max);

            %Create merged bounding boxes in [X Y H W] format
            mergedTextROI = [x1, y1, x2 - x1, y2 - y1];
            
            %Calculate size of labels after updating connected bounding boxes
            labelSizes = histcounts(labelledROI.', max(labelledROI), ...
                'BinMethod', 'integers');

            %Remove single, unconnected bounding boxes
            wordCandidates = labelSizes > 1;
            filteredTextROI = mergedTextROI(wordCandidates, :);

            %Get the bounding boxes of removed objects
            removePixelsROI = mergedTextROI(~wordCandidates, :);
            %Ensure the bounding box isn't empty
            removePixelsROI(all(~removePixelsROI, 2), :) = [];

            %Get the [X Y H W] values of bounding box accounting for real values
            removeMinX = ceil(removePixelsROI(:, 1));
            removeMaxX = round(removeMinX + removePixelsROI(:, 3));
            removeMinY = ceil(removePixelsROI(:, 2));
            removeMaxY = round(removeMinY + removePixelsROI(:, 4));

            %Remove single, unconnected bounding boxes for binary image
            removeROIImage = image;
            for i = 1:size(removePixelsROI, 1)
                %Set pixels to 0 inside bounding boxes that need to be removed
                removeROIImage(removeMinY(i):removeMaxY(i), removeMinX(i):removeMaxX(i)) = 0; 
            end

            %Expand the bounding box vertically to fully contain the text's height 
            pixelExpansion = 2;
            expandedY = filteredTextROI(:, 2) - pixelExpansion;
            expandedH = filteredTextROI(:, 4) + (2 * pixelExpansion);
            %Ensure that ROI is within bounds of the image
            expandedY = max(expandedY, 1);
            expandedH = min(expandedH, obj.h - expandedH);

            %Create expanded bounding boxes in appropriate format
            out = [filteredTextROI(:, 1), expandedY, filteredTextROI(:, 3), ... 
                expandedH];
        end
        
        %Create own f(x) for orientation correction
    end
end

%TODO:
%Maybe outputs should have actual names (not out?)
%Get and Set functions for the image property - will re-calculate h & w
    %Dependant variable?
%Change var names for textGrouping()

