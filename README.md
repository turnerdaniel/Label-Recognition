# Label Recognition

Label Recognition is a MATLAB script which implements an expiry date detection and recognition algorithm. It is capable of recognising expiry dates within a supplied image and allows the text to be saved to a file. 

## Installation

MATLAB 2018a along with the Image Processing and Computer Vision Toolboxes are required to execute this software. You may need to purchase a [license](https://uk.mathworks.com/pricing-licensing.html).

Afterwards, clone the repository using:

```bash
git clone https://github.com/turnerdaniel/Label-Recognition
```

## Usage

Open the MATLAB IDE. Ensure that the directory containing ```script.m``` is selected as the current folder. 

Enter the following in the Command Window to execute the software:

```
script
```
Import an image using the file selection window.

![Image Selection Window](assets/load.png "Image Selection Window")

View the detected dates. 

![Date Detections](assets/detections.png "Date Detections")

They can also be saved to a file.

![Save Detections](assets/save.png "Save Detections")

## Author

Daniel Turner - [turnerdaniel](https://github.com/turnerdaniel/)
