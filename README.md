# Label Recognition

Label Recognition provides a MATLAB application that is capable of recognising expiry dates within a natural image and displaying them back to the user.

The application utilises a fully documented class, `LabelRecogniser`, that implements the expiry date detection and recognition algorithm that is provided with this repository. 

## Prerequisites

This project requires MATLAB, a proprietary language developed by MathWorks, for execution. 

**It is recommended to use the latest stable release of MATLAB**. The performance of releases preceding MATLAB 2018a is not guaranteed.

*Please check with your institution/organisation to see if you are offered a free MATLAB license. Otherwise, see the [licensing options](https://www.mathworks.com/pricing-licensing.html) available.*

## Installation
### Application

1. Navigate to the [latest release](https://github.com/turnerdaniel/Label-Recognition/releases/latest) of this repository.

2. Download the `Label Recognition.mlappinstall` file.

3. Open the file and follow the installation instructions until it completes.

### Class

1. Extract the zip file or clone the repository into the desired directory.

2. Add the parent directory (Label-Recognition) to the path by:
    * Going to Home &rarr; Environment &rarr; Set Path, or
    * Running `addpath [directory]` in the Command Window, or 
    * Having the parent directory selected as the Current Folder.

3. Use in your own MATLAB code.

*Note: Separating the `LabelRecogniser.m` file into its own directory may improve path performance.*

## Usage
### Application

*Ensure that the current folder does not contain `LabelRecognition.m` or `LabelRecogniser.m` files since they could shadow the app entry point.* 

1. Go to Apps Tab &rarr; My Apps &rarr; Label Recognition.

2. Use the GUI window to perform actions such as:

    - Selecting an image
    - Performing date recognition
    - Configuring detection parameters
    - Viewing recognised dates
    - Outputting dates to a file

Auto-Reflow is used to ensure that the app optimally scales to the desired screen size.

### Class

1. Initialise a `LabelRecogniser` object with an image. This can be either a file path or an image matrix.

    ```MATLAB
    lr = LabelRecogniser('samples/good1.jpeg');
    ```

2. Call `recogniseDates()` on the `LabelRecogniser` object to identify expiry dates 
    
    ```MATLAB
    dates = lr.recogniseDates()

    dates = 
        "25 MAR"
    ```

    Alternatively, candidate bounding boxes may be returned too. 
    ```MATLAB
    [dates, bboxes] = lr.recogniseDates()

    dates = 
        "25 MAR"

    bboxes = 
        252.8333    1.0000  106.3333   48.0000
        374.1667  241.5000  165.0000   56.0000
        ...       ...       ...        ...
    ```
    
Further help is provided by using the Command Window to execute:

```MATLAB
doc LabelRecogniser
```

## Demonstration

![GUI Example](https://user-images.githubusercontent.com/35703802/73096390-957dd380-3edc-11ea-826f-2104e02ddd57.png "GUI Example")

Fig 1: MATLAB app successfully identifying the expiry date from an image.

![GUI Auto-Reflow](https://user-images.githubusercontent.com/35703802/73096391-957dd380-3edc-11ea-9b77-c4f500591181.png "GUI Auto-Reflow")

Fig 2: MATLAB app automatically reflowing content to fit onto a smaller screen.

![GUI Error Reporting](https://user-images.githubusercontent.com/35703802/73096392-957dd380-3edc-11ea-9cf9-f6379f626c0b.png "GUI Error Reporting")

Fig 3: MATLAB app displaying Clear error messages that suggest possible fixes.

Alternatively, a demonstration of the `LabelRecogniser` class functionality is provided by the `main.m` script. Execute it in the Command Window using:

```MATLAB
main
```

## Standalone Executable

A standalone executable version of this project was planned to allow non-MATLAB users to make use of the system. However, due to unforeseen complications relating to C/C++ code generation, this task will take longer than expected.

This is due to several functions such as `wiener2`, `detectMSERFeatures`, `imresize`, `imcrop` and `ocr` having  quirks that will require a major rework of the `LabelRecogniser` class due to incompatibilities with the MATLAB Compiler.

Additional details for this problem can be found on the [issues](https://github.com/turnerdaniel/Label-Recognition/issues/9) page. Any contribution will be greatly appreciated.

## Author

Daniel Turner - [turnerdaniel](https://github.com/turnerdaniel/)

## References

##### Stroke Width Transform Algorithm:
Li, Y. and Lu, H. (2012) Scene Text Detection via Stroke Width. In: 21st International Conference on Pattern Recognition (ICPR 2012), Tsukuba, Japan, 11-15 November. IEEE, 681–684. Available from https://ieeexplore.ieee.org/document/6460226 [accessed February 2019].

##### Candidate Text Grouping Algorithm:
MathWorks (2019) Automatically Detect and Recognize Text in Natural Images. Available from https://uk.mathworks.com/help/vision/examples/automatically-detect-and-recognize-text-in-natural-images.html [accessed February 2019].