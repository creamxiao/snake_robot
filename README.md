# Snake-like robot bracing and passive supporting configuration search #
This program is only for the configuration search for Paper "A Search-based Configuration and Motion Planning Algorithm for a Snake-like Robot Performing Load-intensive Operations".

## Prerequisites: ##
**Linux** environment, **OpenCV** and **qpOASES** libraries installed.
### OpenCV:
If you already have your OpenCV installed on you computer, congratulations. Otherwise, skip to [OpenCV installation instructions](#opencv-quick-installation) and remember to come back after you succeeded.

### qpOASES:
The following steps are extracted from [qpOASES manual](https://www.coin-or.org/qpOASES/doc/3.2/manual.pdf). You can chech out the manual for more details.
1. Download [qpOASES](https://github.com/coin-or/qpOASES). You obtained a zipped archive, unpack the archive.
2. A new directory **qpOASES-3.2.1** will be created. From now on we refer to (the full path of) this directory (or the one you used to check out the latest stable branch) by `<qpOASES-dir>`. 
3. Compilation of the qpOASES library **libqpOASES.a** and test examples:
```
    cd <qpOASES-dir>
    make
```
This library libqpOASES.a provides the complete functionality of the qpOASES software package. It can be used by, e.g., linking it against a main function from the examples folder. The make also compiles a couple of test examples; executables are stored within the directory `<qpOASES-dir>/bin`.

4. Running a simple test example:
Among others, an executable called **example1** should have been created; run it in order to test your installation:
```
    cd <qpOASES-dir>/bin
    ./example1
```
If it terminates after successfully solving two QP problems, qpOASES has been successfully installed!

## Installation and Compilation of the program: ##
### Installation: ###
Simply download the whole repository to your local directory. For user's convenience, the third party library [DOSL](https://github.com/subh83/DOSL) is included in this repository. Hereafter we refer to (the full path of) this directory by `<snake_robot-dir>`.

You may need to change the file `<snake_robot-dir>/makefile` prior to the compilation. Depending on where you installed qpOASES library, change the line 35 `BINDIR = <qpOASES-dir>/bin` into your pqOASES installed directory, for example `BINDIR = /home/xiaolong/Documents/qpOASES-3.2.1/bin`.

### Compilation: ###
In terminal, go to the local folder where you put this repository 
```
    cd <snake_robot-dir>
```
compile by running
```
    make snake_11f_preset
```

## Run the program: ##
In the same folder, type in terminal
```
    ./snake_11f_preset
```
No other input needed. During the process of the program, you may be notified to press any key to continue once some picture pops up. Move the focus onto the picutre window and press any key. The output will be in folder `<snake_robot-dir>/outfiles`.

## OpenCV quick installation: ##
Since installing OpenCV merely in terminal is painful, hereafter we are going to install it in a much more convenient way.
1. Open the Linux terminal, input and execute:
```
    sudo apt-get install synaptic
```
2. After the package `synaptic` has been successfully installed, open **Synaptic Package Manager** in your system applications, click **search** button on the top and type in `opencv`. Click **search**.
3. Check all following items from the search results and choose **mark for installation**.

> libcv-dev
libcv2.4
libcvaux-dev
libcvaux2.4
libhighgui-dev
libopencv-calib3d-dev
ibopencv-calib3d2.4v5
libopencv-contrib-dev
libopencv-contrib2.4v5
libopencv-core-dev
libopencv-core2.4v5
libopencv-dev
libopencv-features2d-dev
libopencv-features2d2.4v5
libopencv-flann-dev
libopencv-flann2.4v5
libopencv-gpu-dev
libopencv-gpu2.4v5
libopencv-highgui-dev
libopencv-highgui2.4-deb0
libopencv-imgproc-dev
libopencv-imgproc2.4v5
libopencv-legacy-dev
libopencv-legacy2.4v5
libopencv-ml-dev
libopencv-ml2.4v5
libopencv-objdetect-dev
libopencv-objdetect2.4v5
libopencv-ocl-dev
libopencv-ocl2.4v5
libopencv-photo-dev
libopencv-photo2.4v5
libopencv-stitching-dev
libopencv-stitching2.4v5
libopencv-superres-dev
libopencv-superres2.4v5
libopencv-ts-dev
libopencv-ts2.4v5
libopencv-video-dev
libopencv-video2.4v5
libopencv-videostab-dev
libopencv-videostab2.4v5
libopencv2.4-java
libopencv2.5-jni
opencv-data
3. Click **apply**. Enjoy!

[Go back to Prerequisites](#opencv)
