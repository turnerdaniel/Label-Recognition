# Label Recognition

Label Recognition provides a MATLAB class that implements an expiry date detection and recognition algorithm. It is capable of recognising expiry dates within a supplied image and outputting them to the screen.

## Installation

It is recommended to use the __latest stable release of MATLAB__ to utilise this project. The performance of releases preceding MATLAB 2018a is not guranteed.

This project requires the [Image Processing](https://www.mathworks.com/products/image.html) and [Computer Vision](https://www.mathworks.com/products/computer-vision.html) Toolboxes.

1. Extract the zip file or clone the repository in the desired directory.

2. Add the parent directory (Label-Recognition) to the path:
    * Go to Home &rarr; Environment &rarr; Set Path, or
    * Run `addpath [directory]` in the Command Window, or 
    * Have the parent directory selected as the Current Folder.

*Please check with your institution/organisation to see if you are offered a free MATLAB license. Otherwise, view the licensing options [here](https://www.mathworks.com/pricing-licensing.html).*

## Usage

1. Initialise a `LabelRecogniser` object with an image. This can be either a filepath or an image matrix.

    ```MATLAB
    lr = LabelRecogniser('samples/good1.jpeg');
    ```

2. Call `recogniseDates()` on the `LabelRecogniser` object to identify expiry dates 
    
    ```MATLAB
    dates = lr.recogniseDates()

    dates = 
        "25 MAR"
    ```

    Alternatively, candidate bounding boxes can be returned as well. 
    ```MATLAB
    [dates, bboxes] = lr.recogniseDates()

    dates = 
        "25 MAR"

    bboxes = 
        252.8333    1.0000  106.3333   48.0000
        374.1667  241.5000  165.0000   56.0000
        ...       ...       ...        ...
    ```
    
Further help is provided by the documentation supplied with `LabelRecogniser` which can be accessed by executing:

```MATLAB
doc LabelRecogniser
```

### Demo

![GUI Example](https://user-images.githubusercontent.com/35703802/66256158-f98c5880-e782-11e9-9f90-740adaa96555.png "Image Selection, Date Display and Save As Windows")

A demonstration of the software's functionality is provided by the `main.m` script. Execute it in the Command Window using:

```MATLAB
main
```

This allows for a sample image to be selected, recognised dates to be displayed and for them to be saved to a file when desired. A basic GUI is provided for convenience.

## Author

Daniel Turner - [turnerdaniel](https://github.com/turnerdaniel/)

## References

##### Stroke Width Transform Algorithm:
Li, Y. and Lu, H. (2012) Scene Text Detection via Stroke Width. In: 21st International Conference on Pattern Recognition (ICPR 2012), Tsukuba, Japan, 11-15 November. IEEE, 681–684. Available from https://ieeexplore.ieee.org/document/6460226 [accessed 10 February 2019].

##### Candidate Text Grouping Algorithm:
Mathworks (2019) Automatically Detect and Recognize Text in Natural Images. Available from https://uk.mathworks.com/help/vision/examples/automatically-detect-and-recognize-text-in-natural-images.html [accessed 10 February 2019].