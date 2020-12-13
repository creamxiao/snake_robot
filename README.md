# Snake-like robot bracing and passive supporting configuration search
This program is only for the configuration search for Paper "A Search-based Configuration and Motion Planning Algorithm for a Snake-like Robot Performing Load-intensive Operations".

Prerequisites:
----------------------------------------
_Linux_ environment, _OpenCV_ and _qpOASES_ libraries installed.
### OpenCV:
If you already have your OpenCV installed on you computer, congratulations. Otherwise, skip to [OpenCV installation instructions](#OpenCVquickinstallation).

### qpOASES:
_qpOASES_ download: [Click here](https://github.com/coin-or/qpOASES)

_qpOASES_ manual for installation: [Click here](https://www.coin-or.org/qpOASES/doc/3.2/manual.pdf)

Installation and Compilation of the program:
----------------------------------------
### Installation:
Simply download the whole repository to your local directory. For user's convenience, the third party library [DOSL](https://github.com/subh83/DOSL) is included in this repository.

You may need to change the `makefile` for compilation. Depending on where you installed _qpOASES_ library. Change the line 35 `BINDIR = <qpOASES-install-dir>` to your installed directory, for example `BINDIR = /home/xiaolong/Documents/qpOASES-3.2.1/bin`.

### Compilation:
In terminal, go to the local folder where you put this repository 
```
    cd <snake_robot-dir>
```
compile by running
```
    make snake_11f_preset
```

Run the program:
----------------------------------------
In the same folder, type in terminal
```
    ./snake_11f_preset
```
You may need to press 'space' or any key to continue the program once some picture pops up.

OpenCV quick installation:
----------------------------------------
Since installing OpenCV merely in terminal is painful, hereafter we are going to install it in a much more convenient way.
1. Open the Linux terminal, input and execute:
```
    sudo apt-get install synaptic
```
2. After the package `synaptic` has been successfully installed, open `Synaptic Package Manager` in your system applications, search for: `opencv`
3. Right click to mark the following items in search results for installation:
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
3. Click `apply`. Enjoy!
