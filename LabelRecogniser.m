classdef LabelRecogniser
    %LabelRecogniser Expiry Date Detection and Recognition from an image.
    %
    %   Enables the detection and recognition of expiry dates from images 
    %   of food packaging for use within a date verification system.  
    %
    %   LabelRecogniser(image) initialises the class with an image. This 
    %   can be a file path to an image or a uint8 image matrix.
    %
    %   This class utilises the Image Processing Toolbox.
    %
    % Daniel Turner, 2019.
    % --------------------------------------------------------------------
    
    properties 
        image; % Input image matrix used for recognition
        thresholdDelta = 0.3; % Step size between MSER intensity thresholds
        strokeWidthVariationThreshold = 0.4; % Maximum difference in the width letters strokes
        samplingPoints = 10; % Number of points used for estimating the text orientation
    end
    properties (Access = private)
        height % height of input image
        width % width of input image
    end
    
    methods
        function obj = LabelRecogniser(img)
            %LabelRecogniser Construct an instance of this class from a filepath or uint8 image matrix.
            
            obj.image = img;
        end
        
        function [text, bboxImage] = recogniseDates(obj)
            %recogniseDates Identify the position and textual representation of the dates shown within the image.
            %
            %   text = recogniseDates() provides all of the recognised dates. 
            %   Returns an empty string array if none are found.
            %
            %   [text, image] = recogniseDates() provides all of the recognised
            %   dates and an image showing the bounding boxes of likely date regions.
            
            %Ensure that there is atleast one output
            nargoutchk(1, 2);
            
            %Perform steps to detect and recognise expiry dates from an image
            img = convertGrey(obj, obj.image);
            grey = preProcess(obj, img);
            img = mser(obj, grey);
            img = postProcess(obj, img);
            img = connectedComponentEnhance(obj, img, grey);
            img = geometricFilter(obj, img);
            img = strokeWidthTransform(obj, img);
            [img, bboxes] = textGrouping(obj, img);
            allText = characterRecognition(obj, img, bboxes);
            text = dateMatching(obj, allText);
            
            %If there is more than 1 output argument
            if nargout > 1
                %Overlay bounding boxes on image
                bboxImage = insertShape(obj.image, 'Rectangle', bboxes, 'LineWidth', 3);
            end
        end

        function obj = set.image(obj, value)
            %Setter for image property
            
            %Perform different cases dependent on value's data type
            switch class(value)
                case {'char', 'string'} %filepath
                    try
                        %Attempt to read image from filepath
                        obj.image = imread(value);
                    catch
                        error("LabelRecogniser:BadImageFile", "Cannot read image due to an invalid file type.");
                    end
                    obj = obj.updateImageDimensions();
                    
                case 'uint8' %image matrix
                    %Check that dimensions are sufficient to be an image
                    if ~isvector(value)
                        obj.image = value;
                        obj = obj.updateImageDimensions();
                    else
                        error("LabelRecogniser:BadImageDimensions", "Image dimensions are too small.");
                    end
                    
                otherwise %else
                    error("LabelRecogniser:BadImage", "Invalid data type. Must be a file location or uint8 array.");
            end
        end
        function obj = set.thresholdDelta(obj, value)
            %Setter for thresholdDelta property
            
            %Ensure that value falls within range supported by MSER
            if value > 0 && value <= 100   
                obj.thresholdDelta = value;
            else
                error('LabelRecogniser:ExceededRange', 'The chosen value exeeds the acceptable range of [0 100].');
            end
                
            
        end
        function obj = set.strokeWidthVariationThreshold(obj, value)
            %Setter for strokeWidthVariationThreshold property
            
            %Ensure that value falls within range of possible SWT values
            if value > 0 && value <= 1
                obj.strokeWidthVariationThreshold = value;
            else
                error('LabelRecogniser:ExceededRange', 'The chosen value exeeds the acceptable range of [0 1].');
            end
        end
    end
    methods (Access = private)
        function obj = updateImageDimensions(obj)
            %updateImageDimensions Update the height and width properties according to image dimensions
            
            [obj.height, obj.width, ~] = size(obj.image);
        end
        
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
            out = false(obj.height, obj.width);
            
            %Could make parameters editable?
            mser = detectMSERFeatures(image, 'RegionAreaRange', [150 1500], ...
                'ThresholdDelta', 2.5, 'MaxAreaVariation', 0.2);
            
            %Concatenate pixel coordinates from MSER as Nx2 matrix
            pixels = cell2mat(mser.PixelList);            
            %Convert img co-ordinates to linear image indexes
            ind = sub2ind([obj.height, obj.width], pixels(:,2), pixels(:,1));
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
            adjusted = false(obj.height, obj.width);
            
            for i = 1:numObjects
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
            %geometricFilter Remove unlikely text candidates using geometric properties
            
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
            %strokeWidthTransform Remove unlikley text candidates based upon the width of the characters
            
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
        
        function [image, out] = textGrouping(obj, image)
            %textGrouping Establish bounding boxes around potential dates using a set of rules
            
            %Ensure that there are 2 output arguments
            nargoutchk(2, 2);
            
            %Get the bounding box for each object and convert to usable coordinates
            stats = regionprops(image, 'BoundingBox');
            ROIs = vertcat(stats.BoundingBox);

            %Seperate bounding box into seperate variables for manipulation
            roiX = ROIs(:, 1);
            roiY = ROIs(:, 2);
            roiW = ROIs(:, 3);
            roiH = ROIs(:, 4);

            %Expand ROI by 2/3 the character width in horizontal direction 
            expandedX = roiX - (roiW * (2/3)); 
            expandedW = roiW + ((roiW * (2/3)) * 2);

            %Ensure that ROI is within bounds of the image
            expandedX = max(expandedX, 1);
            expandedW = min(expandedW, obj.width - expandedX);

            %Create expanded bounding boxes
            expandedROI = [expandedX, roiY, expandedW, roiH];
            
            %Calculate the ratio of union between bounding boxes
            overlapRatio = bboxOverlapRatio(expandedROI, expandedROI, 'Union');
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
                boxHeight = roiH(connectedBoxes);

                %Calculate the average height of connected bounding boxes
                meanHeight = mean(boxHeight);
                meanError = meanHeight/2;

                %Find all bounding boxes that have a height that matches the criteria
                validBoxes = boxHeight > meanHeight - meanError & boxHeight < meanHeight + meanError;
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
            x1 = accumarray(labelledROI, expandedX, [], @min);              %    |------------(x2,y2)
            y1 = accumarray(labelledROI, roiY, [], @min);                   %    |               |
            x2 = accumarray(labelledROI, expandedX + expandedW, [], @max);  %    |               |
            y2 = accumarray(labelledROI, roiY + roiH, [], @max);            % (x1,y1)------------|

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
            for i = 1:size(removePixelsROI, 1)
                %Set pixels to 0 inside bounding boxes that need to be removed
                image(removeMinY(i):removeMaxY(i), removeMinX(i):removeMaxX(i)) = 0; 
            end

            %Expand the bounding box vertically to fully contain the text's height 
            expansionValue = 2;
            expandedY = filteredTextROI(:, 2) - expansionValue;
            expandedH = filteredTextROI(:, 4) + (2 * expansionValue);
            %Ensure that ROI is within bounds of the image
            expandedY = max(expandedY, 1);
            expandedH = min(expandedH, obj.height - expandedH);

            %Create expanded bounding boxes in appropriate format
            out = [filteredTextROI(:, 1), expandedY, filteredTextROI(:, 3), ... 
                expandedH];
        end
        
        function roi = orientationCorrection(obj, roi, letters)
            %orientationCorrection Ensure that each word has a horizontal orientation 
            
            %Get the centre of the bounding boxes
            centreX = round(mean([letters(:, 1), letters(:, 1) + letters(:, 3)], 2));
            centreY = round(mean([letters(:, 2), letters(:, 2) + letters(:, 4)], 2));
            
            %Calculate line of best fit coeffecients using the centre of letters
            bestFit = polyfit(centreX, centreY, 1);
            
            %Create linearly spaced vector of 10 sample points from 1 to width of
            %the image
            xPositions = linspace(1, size(roi, 2), obj.samplingPoints);
            
            %Estimate the values of y at x values using the best fit coeffecient
            %to create a line of best fit
            yPositions = polyval(bestFit, xPositions);
            
            %Calculate the line of best fit's angle
            orientation = atan2d(yPositions(obj.samplingPoints) - yPositions(1), ...
                xPositions(obj.samplingPoints) - xPositions(1));
            
            %Don't correct image if it is within 7.5 degrees of 0
            %OCR is capable of reading letters most accurately at ~< 10 degree offset
            if (orientation > 7.5 || orientation < -7.5)
                %Rotate the image by the angle using nearest negithbour interpolation
                roi = imrotate(roi, orientation);
            end
        end
              
        function out = characterRecognition(obj, image, bboxes)
            %characterRecognition Recognise characters within bounding boxes
            
            %Pre-allocate vectors & initialise variables to hold number of bboxes, OCR
            %results and the sampling points for the line of best fit
            numBboxes = size(bboxes, 1);
            out = strings(numBboxes, 1);

            %Loop through text lines
            for i = 1:numBboxes
                %Crop binary image using expanded bounding boxes
                roi = imcrop(image, bboxes(i, :));

                %Remove pixels touching the edge since they are probably not related to text
                roi = imclearborder(roi);

                %Get minimum Bounding Boxes for each letter
                stats = regionprops(roi, 'BoundingBox');

                %Check that there is more than one object in image
                if (size(stats, 1) > 1)
                    %Convert to 4xN matrix for further operations
                    letters = vertcat(stats.BoundingBox);
                    
                    roi = orientationCorrection(obj, roi, letters);
                else 
                    continue
                end
                
                %Perform OCR on the image using MATLAB's Tesseract-OCR 3.02 implementation
                %Specifying 'Block' will ensure that it looks for one or more horizontal text lines
                %Specifying the character set will reduce the chance of confusion with
                %characters that cannot be found in dates
                ocrOutput = ocr(roi, 'TextLayout', 'Block', 'CharacterSet', ...
                    '1234567890ABCDEFGHIJLMNOPRSTUVYabcdefghijlmnoprstuvy/.-:');

                %Store the detected text
                out(i) = ocrOutput.Text;
            end
        end
        
        function out = dateMatching(~, text)
            %dateMatching Identfiy and isolate common date formats found within text
            
            regexTextDate = '(\d{1,2})([\/\\\-\. ]|)(Jan(?:uary)?|Feb(?:ruary)?|Mar(?:ch)?|Apr(?:il)?|May|Jun(?:e)?|Jul(?:y)?|Aug(?:ust)?|Sep(?:tember)?|Oct(?:ober)?|Nov(?:ember)?|Dec(?:ember)?)([\/\\\-\. ]|)((?:\d{2}){0,2})';
            regexTextYear = '^(Jan(?:uary)?|Feb(?:ruary)?|Mar(?:ch)?|Apr(?:il)?|May|Jun(?:e)?|Jul(?:y)?|Aug(?:ust)?|Sep(?:tember)?|Oct(?:ober)?|Nov(?:ember)?|Dec(?:ember)?)([\/\\\-\. ]|)(?:\d{2}){1,2}';
            regexNumeric = '(\d{1,2})([\/\\\-\.])(\d{1,2})(\2)((?:\d{2}){1,2})';

            %Remove newline characters from the end of the text
            text = strip(text, newline);

            %Perform case-insensitive regular expression matching for dates, returning
            %a cell array of matching text
            validTextDate = regexpi(text, regexTextDate, 'match');
            validTextYear = regexpi(text, regexTextYear, 'match');
            validNumeric = regexpi(text, regexNumeric, 'match');

            %concatenate matching text into single string array
            out = string(vertcat(validTextDate{:}, validTextYear{:}, validNumeric{:}));
        end
    end
end

%{ 
TODO:
Maybe outputs should have actual names (not out?)
Could maybe do regionprops on whole image then segement into regions
Docs for properties and use them in code
%} 
