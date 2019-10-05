# Label Recognition Tests

This test module is provided as part of the `LabelRecogniser` class and is designed to evaluate its performance in a variety of different scenarios.

## Installation

No installation necessary. The test module imports `LabelRecogniser` from the parent class when executed.

## Usage

The test module provides 2 test classes: 

* `LabelRecogniserTests` uses MATLAB's unit test framework to validate the software's functionality.
* `LabelRecogniserVerfication` assesses the performance of the software on the dataset.

### LabelRecogniserTests

1. Run the tests by executing:
    ```MATLAB
    run(LabelRecogniserTests)
    ```

2. Wait until all unit tests complete:
    ```MATLAB
    Running LabelRecogniserTests
    .............
    Done LabelRecogniserTests
    ```
3. Review results:
    ```MATLAB
    Totals:
        13 Passed, 0 Failed, 0 Incomplete.
        3.652 seconds testing time.
    ```

### LabelRecogniserVerification
__WARNING:__ This test take can take upwards of 10 minutes to complete on slow systems.

1. Run the tests by executing
    ```MATLAB
    LabelRecogniserVerification
    ````

2. Wait for `LabelRecogniser.recogniseDates()` to be ran on the 500 images in the dataset.

3. Review the metrics:
    ```MATLAB
    LabelRecogniserVerification with properties:

                  Precision: 0.0980
                     Recall: 0.9820
        RecognitionAccuracy: 0.8880
            OverallAccuracy: 0.8740
                MinDuration: 0.2721
                MaxDuration: 2.2278
               MeanDuration: 1.8752
             StdDevDuration: 0.4991
           StdErrorDuration: 0.0223
    ```

Further help is provided by the documentation supplied with `LabelRecogniserVerification` which can be accessed by executing:

```MATLAB
doc LabelRecogniserVerification
```

## Inconsistencies

The Recognition Accuracy results from `LabelRecogniserVerification` can sometimes be unreliable due to the difficulty in discerning what classes as a 'detection'.

Officially, it is when the Intersection over Union (IoU) of a predicted and the ground truth bounding box exceeds 0.5. 

However, sometimes a date can recognised from bounding box which has not been classed as detected. e.g. The predicted bounding box is too big therefore the IoU does not exceed 0.5.

## Dataset

The dataset consists of a sample of 500 images that were pseudo-randomly selected from a collection of 9,651 images that represent the population.

These images consisted of photos taken of food packaging in a natural environment that had variances in illumination, scale, orientation, font and perspective.

Images are labelled numerically within the dataset directory.

## Continuous Integration

It would be ideal for this project to implement Continuous Integration (CI) to ensure reliable, functional and consistent code. 

However, a number of factors prevent from being achievable:

* MATLAB is a proprietary language and is therefore not available on most CI services. 
* Open-source alternatives such as GNU Octave lack compatability with MATLAB's unit test framework and certain Toolbox functions.
* Limited documentation and support for workaround methods.

Therefore, no form of Continuous Integration is currently being used. 

If you have an idea for implementing a complete CI solution, please open an Issue. 