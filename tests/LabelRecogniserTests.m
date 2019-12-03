classdef LabelRecogniserTests < matlab.unittest.TestCase
    %LabelRecogniserTests Test class for LabelRecogniser that verifies intended functionality.
    %   
    %   run(LabelRecogniserTests) executes the unit tests and displays the
    %   results.
    
    methods (TestClassSetup)
        function addLabelRecogniserToPath(testCase)
            %Add LabelRecogniser & sample images to path
            p = addpath('..');
            %Restore path when finished
            testCase.addTeardown(@path, p);
        end
    end
    
    methods (Test)
        
        function testNoConstructor(testCase)
            lr = LabelRecogniser();
            testCase.assertEqual(lr.image, []);
        end

        function testConstructorAsPath(testCase)
            I = imread('images/valid.jpeg');
            lr = LabelRecogniser('images/valid.jpeg');
            testCase.assertEqual(lr.image, I);
        end
        
        function testConstructorAsImage(testCase)
            I = imread('images/valid.jpeg');
            lr = LabelRecogniser(I);
            testCase.assertEqual(lr.image, I);
        end
        
        function testConstructorInvalidImage(testCase)
            %Use anonymous function handle to create object
            testCase.assertError(@()LabelRecogniser('notanimage.jpeg'), ...
                'LabelRecogniser:BadImageFile');
        end
        
        function testConstructorValues(testCase)
            expTd = 2.5;
            expSwvt = 0.4;
            expOt = 7.5;
            
            lr = LabelRecogniser('images/valid.jpeg');
            testCase.assertEqual(lr.thresholdDelta, expTd);
            testCase.assertEqual(lr.strokeWidthVariationThreshold, expSwvt);
            testCase.assertEqual(lr.orientationThreshold, expOt);            
        end
        
        function testChangeThresholdDeltaSuccess(testCase)
            expTd = 5;
            
            lr = LabelRecogniser('images/valid.jpeg');
            lr.thresholdDelta = expTd;
            testCase.assertEqual(lr.thresholdDelta, expTd);
        end
        
        function testChangeThresholdDeltaFailure(testCase)
            lr = LabelRecogniser('images/valid.jpeg');
            testCase.assertError(@setThresholdDelta, ... 
                'LabelRecogniser:ExceededRange');
            
            function setThresholdDelta()
                lr.thresholdDelta = 250;
            end
        end
        
        function testChangeStrokeWidthVariationThresholdSuccess(testCase)
            expSwvt = 0.5;
            
            lr = LabelRecogniser('images/valid.jpeg');
            lr.strokeWidthVariationThreshold = expSwvt;
            testCase.assertEqual(lr.strokeWidthVariationThreshold, expSwvt);
        end
        
        function testChangeStrokeWidthVariationThresholdFailure(testCase)
            lr = LabelRecogniser('images/valid.jpeg');
            testCase.assertError(@setStrokeWidthVariationThreshold, ... 
                'LabelRecogniser:ExceededRange');
            
            function setStrokeWidthVariationThreshold()
                lr.strokeWidthVariationThreshold = 2;
            end
        end
        
        function testChangeOrientationThresholdSuccess(testCase)
            expOt = 15;
            
            lr = LabelRecogniser('images/valid.jpeg');
            lr.orientationThreshold = expOt;
            testCase.assertEqual(lr.orientationThreshold, expOt);
        end
        
        function testChangeOrientationThresholdFailure(testCase)
            lr = LabelRecogniser('images/valid.jpeg');
            testCase.assertError(@setOrientationThreshold, ... 
                'LabelRecogniser:ExceededRange');
            
            function setOrientationThreshold()
                lr.orientationThreshold = 180;
            end
        end
        
        function testRecogniseDatesSuccess(testCase)
            expDate = "01 JAN 2020";
            
            lr = LabelRecogniser('images/valid.jpeg');
            date = lr.recogniseDates();
            testCase.assertEqual(date, expDate);
        end
        
        function testRecogniseDatesSuccessWithBboxOutput(testCase)
            expDate = "01 JAN 2020";
            
            lr = LabelRecogniser('images/valid.jpeg');
            [date, bbox] = lr.recogniseDates();
            testCase.assertEqual(date, expDate);
            testCase.assertNotEmpty(bbox);
        end
        
        function testRecogniseDatesFailure(testCase)
            lr = LabelRecogniser('images/invalid.jpeg');
            date = lr.recogniseDates();
            testCase.assertEmpty(date);
        end
        
        function testRecogniseDatesFailureNoImage(testCase)
            lr = LabelRecogniser();
            testCase.assertError(@()lr.recogniseDates(), ... 
                'LabelRecogniser:NoImage');
        end
    end
end

