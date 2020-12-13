# Snake-like robot bracing and passive supporting configuration search
This program is for the configuration search of Paper "A Search-based Configuration and Motion Planning Algorithm for a Snake-like Robot Performing Load-intensive Operations".

Prerequisites:
----------------------------------------
_Linux_ environment, _OpenCV_ and _qpOASES_ libraries installed.

_qpOASES_ download: [Click here](https://github.com/coin-or/qpOASES)

_qpOASES_ manual for installation: [Click here](https://www.coin-or.org/qpOASES/doc/3.2/manual.pdf)

Installation and Compilation of the program:
----------------------------------------
**Installation:**
Simply download the whole repository to your local directory. For user's convenience, the third party library [DOSL](https://github.com/subh83/DOSL) is included in this repository.

You may need to change the `makefile` for compilation. Depending on where you installed _qpOASES_ library. Change the line 35 `BINDIR = <qpOASES-install-dir>` to your installed directory, for example `BINDIR = /home/xiaolong/Documents/qpOASES-3.2.1/bin`.

**Compilation:**
Go to the local folder where you put this repository in terminal, compile by running
```
    make snake_11f_preset
```

Run the program:
----------------------------------------
In the same folder, type in terminal
```
    ./snake_11f_preset
```
