# Snake-like robot bracing and passive supporting configuration search #
This program is only for the configuration search for Paper "A Search-based Configuration and Motion Planning Algorithm for a Snake-like Robot Performing Load-intensive Operations". The included setups (maps and start/goal poses) are only for demonstrating the optimized performance with the pre-computated cost/heuristic values.

## Prerequisites: ##
**Linux** environment, **OpenCV** and **qpOASES** libraries installed.
### OpenCV: ###
This program runs on OpenCV 3.X. If you already have this version installed on you computer, congratulations. Otherwise, skip to [OpenCV installation instructions](#opencv-quick-installation) to install OpenCV. Remember to come back after you succeeded. If you wouldn't want to mess with your existing other version of OpenCV or you have trouble in installing OpenCV, you can try running it in Docker which provided an independent shell for the program to run. Click [run in Docker](#run-in-docker).

The installation instruction of library **qpOASES** is in the next section.

## Installation and Compilation of the program: ##
### Installation: ###
Simply download the whole repository of `snake_robot` to your local directory and unzip it (make sure this local directory **has no space** in it, otherwise you would have problem in compiling the library **qpOASES**). Or you can clone the snake_robot repoitory by running:
```
    git clone https://github.com/creamxiao/snake_robot.git
```
For reviewers' convenience, the third party library [DOSL](https://github.com/subh83/DOSL) and [qpOASES](https://github.com/coin-or/qpOASES) are included in this repository. Hereafter we refer to (the full path of) this directory by `<snake_robot-dir>`.
### qpOASES: ###
The following steps are extracted from [qpOASES manual](https://www.coin-or.org/qpOASES/doc/3.2/manual.pdf). You can chech out the manual for more details.
1. Since you have downloaded the included **qpOASES**, go to the folder by running this in terminal:
```
    cd <snake_robot-dir>/qpOASES-3.2.1
```
2. Compilation of the qpOASES library **libqpOASES.a** and test examples:
```
    make
```
This library libqpOASES.a provides the complete functionality of the qpOASES software package. It can be used by, e.g., linking it against a main function from the examples folder. The make also compiles a couple of test examples; executables are stored within the directory `<snake_robot-dir>/qpOASES-3.2.1/bin`.

3. Running a simple test example:
Among others, an executable called **example1** should have been created; run it in order to test your installation:
```
    cd <snake_robot-dir>/qpOASES-3.2.1/bin
    ./example1
```
If it terminates after successfully solving two QP problems, qpOASES has been successfully installed!

### Compilation: ###
In terminal, go to the local folder where you put this repository 
```
    cd <snake_robot-dir>
```
compile by running
```
    make snake_config_search_precal
```

## Run the program: ##
In the same folder, type in terminal
```
    ./snake_config_search_precal
```
When there's no other arguments, the default setup is Figure 23a. During the process of the program, once `Display window` pops up and it notifies you to press any key in the terminal, click on the window (move the focus onto the window) and press any key to continue. The output will be in folder `<snake_robot-dir>/outfiles`.

If you want to run it on other setups, add an argument after the programe name, separated with a space. For example, run
```
    ./snake_config_search_precal 26c
```
for Figure 26c. This repository incluces setups for Figure 23a-23h, Figure 26a-26f. The search results and their data may be different from those in the paper. All setup information is stored in folder`<snake_robot-dir>/exptfiles`.
## OpenCV quick installation: ##
On Ubuntu, you can run:

```
sudo apt update
sudo apt install libopencv-dev python3-opencv
```

Since installing OpenCV merely in terminal is painful, hereafter we are going to install it in a much more convenient way. WARNING: This may install the latest version of OpenCV which could lead to some issues.

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

## Run in Docker: ##
The following steps are extracted from online contents. To learn more details, see [Install OpenCV Docker Image on Ubuntu, MacOS or Windows](https://www.learnopencv.com/install-opencv-docker-image-ubuntu-macos-windows/).
1. Install Docker by running in terminal
```
    sudo apt-get install docker.io
```
2. Download and install Docker image of OpenCV, run
```
    sudo docker pull spmallick/opencv-docker:opencv
```
The image's credit to [**spmallick**](https://hub.docker.com/r/spmallick/opencv-docker).

3. You may need to enable Docker to display (in case the display window doesn't show up when you are running the search program). Run
```
    sudo xhost +local:docker
```
[Reference here](https://forums.docker.com/t/start-a-gui-application-as-root-in-a-ubuntu-container/17069).

4. Launch the image, run
```
    sudo docker run -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -p 5000:5000 -p 8888:8888 -it spmallick/opencv-docker:opencv /bin/bash
    cd ~/
```
**If successful, you are now in a Docker window. Follow [these steps](#installation-and-compilation-of-the-program) to install and run the program. Make sure you are doing it all in docker window.** 

To exit your docker window, type:
```
    exit
```
After exiting the docker window, to save your docker setup so you can access later, do the following:
```
    sudo docker commit CONTAINER_ID NAME_OF_DOCKER_IMAGE
```
![screenshot](https://www.learnopencv.com/wp-content/uploads/2018/09/commit-changes-to-docker-image.png)
where **CONTAINER_ID** is the text following **root@** in your docker container. Also, note that Container ID will vary every time you use docker run to create a new container. **NAME_OF_DOCKER_IMAGE** is what you want it to be called.
Then the next time you want to run the code, you just need to type:
```
    sudo docker run -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -p 5000:5000 -p 8888:8888 -it NAME_OF_DOCKER_IMAGE /bin/bash
```
and you should be able to go to:
```
cd ~/snake_robot
```
then execute the program without re-compiling.
