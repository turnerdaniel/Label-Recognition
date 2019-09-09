# Label Recognition

Label Recognition is a MATLAB script which implements an expiry date detection and recognition algorithm. It is capable of recognising expiry dates within a supplied image and allows the text to be saved to a file. 

## Installation

MATLAB 2018a along with the Image Processing and Computer Vision Toolboxes are required to execute this software. You may need to purchase a [license](https://uk.mathworks.com/pricing-licensing.html).

Afterwards, clone the repository using:

```bash
git clone https://github.com/turnerdaniel/Label-Recognition
```

## Usage

Open the MATLAB IDE. Ensure that the directory containing ```main.m``` is selected as the current folder. 

Enter the following in the Command Window to execute the software:

```
main
```
Import an image using the file selection window.

![Image Selection Window](https://user-images.githubusercontent.com/35703802/64533138-d220a800-d30a-11e9-9212-be1391bef9b5.png "Image Selection Window")

View the detected dates. 

![Date Detections](https://user-images.githubusercontent.com/35703802/64533136-d220a800-d30a-11e9-9733-5dae826559a6.png "Date Detections")

They can also be saved to a file.

![Save Detections](https://user-images.githubusercontent.com/35703802/64533139-d220a800-d30a-11e9-848e-70718e706c77.png "Save Detections")

## Author

Daniel Turner - [turnerdaniel](https://github.com/turnerdaniel/)

## References

##### Stroke Width Transform Algorithm:
Li, Y. and Lu, H. (2012) Scene Text Detection via Stroke Width. In: 21st International Conference on Pattern Recognition (ICPR 2012), Tsukuba, Japan, 11-15 November. IEEE, 681–684. Available from https://ieeexplore.ieee.org/document/6460226 [accessed 10 February 2019].

##### Candidate Text Grouping Algorithm:
Mathworks (undated) Automatically Detect and Recognize Text in Natural Images. Available from https://uk.mathworks.com/help/vision/examples/automatically-detect-and-recognize-text-in-natural-images.html [accessed 10 February 2019].