# Change these if needed or pass as input parameter
# PREFIX = /usr/local

# ================================

# INCLUDE FOLDER
# --------------

ifneq ("$(wildcard /usr/local/include/dosl)","")
    DOSL_FOLDER = /usr/local/include
else ifneq ("$(wildcard ../dosl)","")
    DOSL_FOLDER = ..
else
	DOSL_FOLDER = $(PREFIX)/include
endif

UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
	OS_String = "Compiling on Linux"
	OS_FLAG = 1

	# C++ Compiler command
	CC = g++
	# C++ Compiler options
	CXXFLAGS = -O3 -pipe -DNDEBUG -Wparentheses -Wreturn-type -Wcast-qual -Wall -Wpointer-arith -Wwrite-strings -Wconversion -Wno-unknown-pragmas -Wno-long-long   -DCLP_BUILD
	# additional C++ Compiler options for linking
	CXXLINKFLAGS = -Wl,--rpath 
	
	# C++ Compiler options (QP)
	DEF_SOLVER = SOLVER_NONE
	CXXFLAGSQP = -Wall -pedantic -Wshadow -Wfloat-equal -O3 -Wconversion -Wsign-conversion -fPIC -DLINUX -D__USE_LONG_INTEGERS__ -D__USE_LONG_FINTS__ -D${DEF_SOLVER} -D__NO_COPYRIGHT__
	# additional C++ Compiler options for linking (QP)
	
	# change this to your installed qpOASES directory, for example: BINDIR = /home/xiaolong/Documents/qpOASES-3.2.1/bin
	BINDIR = <qpOASES-dir>/bin
	
	CXXLINKFLAGSQP = -L${BINDIR} -Wl,-rpath=${BINDIR} ${LINKHSL} -lqpOASES

	# libraries to link against when building qpOASES .so files
	# SRCDIR = /home/xiaolong/Documents/qpOASES-3.2.1/src
	# LIB_LAPACK = ${SRCDIR}/LAPACKReplacement.o
	# LIB_BLAS =   ${SRCDIR}/BLASReplacement.o
	LIB_SOLVER =
	LINKHSL =
	LINK_LIBRARIES = ${LIB_LAPACK} ${LIB_BLAS} -lm ${LIB_SOLVER}
	# how to link against the qpOASES shared library
	QPOASES_LINK = -L${BINDIR} -Wl,-rpath=${BINDIR} ${LINKHSL} -lqpOASES
	INC_QP =

endif

INC_DOSL = -I$(DOSL_FOLDER)
INC_LOCAL = -I. -Iinclude

# --------------------------------------------
# DOSL-specific

DOSL_ALGORITHM_LIST = $(shell find $(DOSL_FOLDER)/dosl/planners -type f -name '*.tcc' -exec basename {} .tcc \;) # AStar SStar
DOSL_ALGORITHM = AStar

DOSL_INTRO_STRING = "Including DOSL from '\033[33m$(DOSL_FOLDER)/\033[39m'\nUsing algorithm '\033[33m$(DOSL_ALGORITHM)\033[39m' [\033[33mAvailable algorithms: $(DOSL_ALGORITHM_LIST); To change algorithm use 'make <target> DOSL_ALGORITHM=<algorithm>'\033[39m].\n"

# --------------------------------------------
# common flags
CXXFLAGS += -O3 -pipe -DNDEBUG -Wparentheses -Wreturn-type -Wcast-qual -Wall -Wpointer-arith -Wwrite-strings -Wconversion -Wno-unknown-pragmas -Wno-long-long   -DCLP_BUILD
CFLAGS = -std=gnu++11 -g -Og
WARNS = -w


LIBS = -lm
LIBS_OPENCV = `pkg-config opencv --cflags --libs`  #-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_shape
LIBS_OPENGL = -lglut -lGLU -lGL -lXi -lXmu -lX11 -lXext
LIB_ARMADILLO = -larmadillo
LIBS_THREAD = -pthread

# --------------------------------------------

.PHONY: snake_config_search_precal
snake_config_search_precal:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXLINKFLAGSQP) $(CXXFLAGSQP) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_QP) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG) $< ${QPOASES_LINK} ${LINK_LIBRARIES}

# --------------------------------------------

all: all_tests all_examples

clean:
	rm $(prog)
	rm $(prog).o
