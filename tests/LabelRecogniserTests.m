classdef LabelRecogniserTests < matlab.unittest.TestCase
    %LabelRecogniserTests Test class for LabelRecogniser that verifies intended functionality.
    %   
    %   TODO: 
    %   Need a 500 image test function in seperate file
    %   Cleanup samples not in path workaround if possible
    
    methods (TestClassSetup)
        function addLabelRecogniserToPath(testCase)
            p = addpath('..', '../samples');
            testCase.addTeardown(@path, p);
        end
    end
    
    methods (Test)
        %setup and teardown of path so can be in own folder
        
        function testConstructorAsPath(testCase)
            I = imread('samples/good1.jpeg');
            lr = LabelRecogniser('samples/good1.jpeg');
            testCase.assertEqual(lr.image, I);
        end
        
        function testConstructorAsImage(testCase)
            I = imread('samples/good1.jpeg');
            lr = LabelRecogniser(I);
            testCase.assertEqual(lr.image, I);
        end
        
        function testConstructorValues(testCase)
            expTd = 2.5;
            expSwvt = 0.4;
            expOt = 7.5;
            
            lr = LabelRecogniser('samples/good1.jpeg');
            testCase.assertEqual(lr.thresholdDelta, expTd);
            testCase.assertEqual(lr.strokeWidthVariationThreshold, expSwvt);
            testCase.assertEqual(lr.orientationThreshold, expOt);            
        end
        
        function testRecogniseDatesSuccess(testCase)
            expDate = "25 MAR";
            
            lr = LabelRecogniser('samples/good1.jpeg');
            date = lr.recogniseDates();
            testCase.assertEqual(date, expDate);
        end
        
        function testRecogniseDatesFailure(testCase)
            
            lr = LabelRecogniser('samples/bad1.jpeg');
            date = lr.recogniseDates();
            testCase.assertEmpty(date);
        end
    end
end

