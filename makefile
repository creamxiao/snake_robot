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
	#CXXFLAGS = -DNETLIBDIR=\"`PKG_CONFIG_PATH=/home/xiaolong/coin-Clp/lib64/pkgconfig:/home/mecad/coin-Clp/lib/pkgconfig:/home/xiaolong/coin-Clp/share/pkgconfig: pkg-config --variable=datadir coindatanetlib`\"
	# additional C++ Compiler options for linking
	CXXLINKFLAGS = -Wl,--rpath -Wl,/home/xiaolong/coin-Clp/lib
	# COIN-OR Include directories
	INC_COIN = `PKG_CONFIG_PATH=/home/xiaolong/coin-Clp/lib64/pkgconfig:/home/xiaolong/coin-Clp/lib/pkgconfig:/home/xiaolong/coin-Clp/share/pkgconfig: pkg-config --cflags clp`
	# COIN-OR Linker flags
	LIBS_COIN = `PKG_CONFIG_PATH=/home/xiaolong/coin-Clp/lib64/pkgconfig:/home/xiaolong/coin-Clp/lib/pkgconfig:/home/xiaolong/coin-Clp/share/pkgconfig: pkg-config --libs clp`

	# C++ Compiler options (QP)
	DEF_SOLVER = SOLVER_NONE
	CXXFLAGSQP = -Wall -pedantic -Wshadow -Wfloat-equal -O3 -Wconversion -Wsign-conversion -fPIC -DLINUX -D__USE_LONG_INTEGERS__ -D__USE_LONG_FINTS__ -D${DEF_SOLVER} -D__NO_COPYRIGHT__
	# additional C++ Compiler options for linking (QP)
	BINDIR = /home/xiaolong/Documents/qpOASES-3.2.1/bin
	CXXLINKFLAGSQP = -L${BINDIR} -Wl,-rpath=${BINDIR} ${LINKHSL} -lqpOASES

	# libraries to link against when building qpOASES .so files
	SRCDIR = /home/xiaolong/Documents/qpOASES-3.2.1/src
	LIB_LAPACK = ${SRCDIR}/LAPACKReplacement.o
	LIB_BLAS =   ${SRCDIR}/BLASReplacement.o
	LIB_SOLVER =
	LINKHSL =
	LINK_LIBRARIES = ${LIB_LAPACK} ${LIB_BLAS} -lm ${LIB_SOLVER}
	# how to link against the qpOASES shared library
	QPOASES_LINK = -L${BINDIR} -Wl,-rpath=${BINDIR} ${LINKHSL} -lqpOASES
	INC_QP =

else #Mac
	OS_String = "Compiling on Mac"
	OS_FLAG = 2

	# C++ Compiler command
	CC = clang++

	# C++ Compiler options
	CXXFLAGS = -DSAMPLEDIR=\"`PKG_CONFIG_PATH=/Users/xiaolong/Dropbox/DOSL-master/coin-Clp/lib64/pkgconfig:/Users/xiaolong/Dropbox/DOSL-master/coin-Clp/lib/pkgconfig:/Users/xiaolong/Dropbox/DOSL-master/coin-Clp/share/pkgconfig: pkg-config --variable=datadir coindatasample`\"

	# additional C++ Compiler options for linking
	CXXLINKFLAGS =

	# COIN-OR Include directories
	INC_COIN = `PKG_CONFIG_PATH=/Users/xiaolong/Dropbox/DOSL-master/coin-Clp/lib64/pkgconfig:/Users/xiaolong/Dropbox/DOSL-master/coin-Clp/lib/pkgconfig:/Users/xiaolong/Dropbox/DOSL-master/coin-Clp/share/pkgconfig: pkg-config --cflags clp`
	# COIN-OR Linker flags
	LIBS_COIN = `PKG_CONFIG_PATH=/Users/xiaolong/Dropbox/DOSL-master/coin-Clp/lib64/pkgconfig:/Users/xiaolong/Dropbox/DOSL-master/coin-Clp/lib/pkgconfig:/Users/xiaolong/Dropbox/DOSL-master/coin-Clp/share/pkgconfig: pkg-config --libs clp`


	# C++ Compiler options (QP)
	SYSROOT = /Applications/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk
	SDK = -isysroot ${SYSROOT} -stdlib=libc++
	CXXFLAGSQP = ${SDK} -Wall -pedantic -Wshadow -Wfloat-equal -Wconversion -Wsign-conversion -O3 -fPIC -DLINUX
	SHARED = -dynamiclib ${SDK} -lgcc_s.10.5 -ldylib1.o

	# additional C++ Compiler options for linking (QP)
	BINDIR = /Users/xiaolong/Dropbox/DOSL-master/snake
	CXXLINKFLAGSQP =

	# libraries to link against when building qpOASES .so files
	SRCDIR = /Users/xiaolong/Documents/qpOASES-3.2.1/src

	# system or replacement BLAS/LAPACK
	LIB_LAPACK = ${SRCDIR}/LAPACKReplacement.o
	LIB_BLAS =   ${SRCDIR}/BLASReplacement.o
	LA_DEPENDS = ${LIB_LAPACK} ${LIB_BLAS}

	LIB_SOLVER =
	DEF_SOLVER = SOLVER_NONE
	LINK_LIBRARIES = ${LIB_LAPACK} ${LIB_BLAS} -lm ${LIB_SOLVER}
	LINK_LIBRARIES_WRAPPER = -lm ${LIB_SOLVER}

	# how to link against the qpOASES shared library
	QPOASES_LINK = -L${BINDIR}  -lqpOASES -L${SYSROOT}/usr/lib/System -lgcc_s.10.5 -lcrt1.o
	QPOASES_LINK_WRAPPER = -L${BINDIR} -lqpOASES_wrapper

	# link dependencies when creating executables
	LINK_DEPENDS = ${LA_DEPENDS} ${BINDIR}/libqpOASES.${LIBEXT} ${BINDIR}/libqpOASES.${DLLEXT}
	LINK_DEPENDS_WRAPPER = ${BINDIR}/libqpOASES_wrapper.${LIBEXT} ${BINDIR}/libqpOASES_wrapper.${DLLEXT}

	#include
	IDIR = /Users/xiaolong/Documents/qpOASES-3.2.1/include
	INC_QP = -I$(IDIR)


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

basic_examples: minimal2d_PathPlanning_AStar simple2d_PathPlanning
cv_examples:    demo2d_PathPlanning  map2d_PathPlanning  homotopy2d_PathPlanning graph_search graph_search_coverpoints_maxarea graph_search_workspace graph_search_movingob graph_search_cablecontrol graph_search_cablecontrol_v2 graph_search_multitask graph_search_multitask_v2 config_search

all_examples: basic_examples cv_examples

.PHONY: snake_0
snake_0:
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_THREAD) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) $(LIB_ARMADILLO)

.PHONY: snake_1_ssh
snake_1_ssh:
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_THREAD) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) $(LIB_ARMADILLO)

.PHONY: snake_2_rrt
snake_2_rrt:
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_THREAD) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) $(LIB_ARMADILLO)
.PHONY: snake_3_brace
snake_3_brace:
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_THREAD) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) $(LIB_ARMADILLO)
.PHONY: snake_4_local
snake_4_local:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_THREAD) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) $(LIB_ARMADILLO)
.PHONY: snake_5_gravity_torque
snake_5_gravity_torque:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_THREAD) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) $(LIB_ARMADILLO)
.PHONY: snake_6_individual_torque
snake_6_individual_torque:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_THREAD) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) $(LIB_ARMADILLO)
.PHONY: snake_7_heuristic_wall_factor
snake_7_heuristic_wall_factor:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM)
.PHONY: snake_7b_only_torque_at_the_end
snake_7b_only_torque_at_the_end:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM)
.PHONY: snake_8_mistakenly_fast
snake_8_mistakenly_fast:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM)
.PHONY: snake_8b_oscillate_cost_function
snake_8b_oscillate_cost_function:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM)
.PHONY: snake_9_Astar_2nd_search
snake_9_Astar_2nd_search:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXFLAGS) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG)
.PHONY: snake_9b_config_test
snake_9b_config_test:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXLINKFLAGSQP) $(CXXFLAGSQP) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) -o $@ $@.cpp ${QPOASES_LINK} ${LINK_LIBRARIES} $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG)
.PHONY: snake_9c_all_qp
snake_9c_all_qp:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXLINKFLAGSQP) $(CXXFLAGSQP) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) $(INC_QP) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG) $< ${QPOASES_LINK} ${LINK_LIBRARIES}
.PHONY: snake_10_drill_motion
snake_10_drill_motion:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXFLAGS) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG)
.PHONY: snake_11_new_map_for_paper
snake_11_new_map_for_paper:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXLINKFLAGSQP) $(CXXFLAGSQP) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) $(INC_QP) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG) $< ${QPOASES_LINK} ${LINK_LIBRARIES}
.PHONY: snake_11d_skeleton_optimized
snake_11d_skeleton_optimized:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXLINKFLAGSQP) $(CXXFLAGSQP) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL)  $(INC_QP) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG) $< ${QPOASES_LINK} ${LINK_LIBRARIES}
.PHONY: snake_11e_revised_functions
snake_11e_revised_functions:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXLINKFLAGSQP) $(CXXFLAGSQP) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) $(INC_QP) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG) $< ${QPOASES_LINK} ${LINK_LIBRARIES}
.PHONY: snake_11f_preset
snake_11f_preset:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXLINKFLAGSQP) $(CXXFLAGSQP) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_QP) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG) $< ${QPOASES_LINK} ${LINK_LIBRARIES}
.PHONY: snake_11f_no_preset
snake_11f_no_preset:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXLINKFLAGSQP) $(CXXFLAGSQP) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_QP) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG) $< ${QPOASES_LINK} ${LINK_LIBRARIES}
.PHONY: snake_11g_result_influence
snake_11g_result_influence:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXLINKFLAGSQP) $(CXXFLAGSQP) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) $(INC_QP) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG) $< ${QPOASES_LINK} ${LINK_LIBRARIES}
.PHONY: snake_11_transformation
snake_11_transformation:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXLINKFLAGSQP) $(CXXFLAGSQP) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) $(INC_QP) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG) $< ${QPOASES_LINK} ${LINK_LIBRARIES}
.PHONY: snake_12_animation
snake_12_animation:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXLINKFLAGSQP) $(CXXFLAGSQP) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) $(INC_QP) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG) $< ${QPOASES_LINK} ${LINK_LIBRARIES}
.PHONY: snake_12b_animation_irregularmap
snake_12b_animation_irregularmap:
	@printf $(OS_String)
	@printf "\nNow compiling '$@'...\n"
	@printf $(DOSL_INTRO_STRING)
	$(CC) $(CXXLINKFLAGS) $(CXXLINKFLAGSQP) $(CXXFLAGSQP) $(CFLAGS) $(WARNS) $(INC_LOCAL) $(INC_DOSL) $(INC_COIN) $(INC_QP) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV) $(LIBS_COIN) -D_DOSL_ALGORITHM=$(DOSL_ALGORITHM) -D_OS_FLAG=$(OS_FLAG) $< ${QPOASES_LINK} ${LINK_LIBRARIES}
# --------------------------------------------

all_tests: cv_test json_test

.PHONY: cv_test
cv_test:
	@printf "\nNow compiling '$@'...\n"
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) -o $@ $@.cpp $(LIBS) $(LIBS_OPENCV)

.PHONY: json_test
json_test:
	@printf "\nNow compiling '$@'...\n"
	$(CC) $(CFLAGS) $(WARNS) $(INC_LOCAL) -o $@ $@.cpp $(LIBS)

# --------------------------------------------

all: all_tests all_examples

clean:
	rm $(prog)
	rm $(prog).o
