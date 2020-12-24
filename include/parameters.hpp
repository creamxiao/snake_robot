#ifndef PARA
#define PARA

#define scale_select 1 //choose scale

#define C4 0.3 // coefficient for heuristic function
#define EL 0.25 // expansion discrtization for passive support configuration search

#if scale_select == 1
  #define OBSTHRESHOLD 100
  #define PLOT_SCALE 3
  #define VERTEX_SIZE 1
  #define LINE_THICKNESS 1 // CV_FILLED
  #define AA 8.0 // non-extentable length
  #define L 8.25 // extentable length
#else
  #define PLOT_SCALE 5
  #define VERTEX_SIZE 1
  #define LINE_THICKNESS 1 // CV_FILLED
  #define AA 2.5
  #define L 2.5858
  #define A_H 3.25
  #define A_L 1.7929
#endif

// physic/mechanic parameters
#define MAXFORCE 750
#define DRILLFORCE 250
#define default_MU 0.3 // friction coefficient
#define LINKMASS 0.5 // in kg
#define HEADMASS 1 // in kg
#define MAXTORQUE 65 // Nm
#define GRAVITYDIR double g[2] = {0, 9.8} // Gravity acceleration, downwards
#define P2M_SCALE 2.5 / 8 * 0.0254 // scale of length in pixels to metric (meters)

#define INFINITESIMAL_DOUBLE  1e-6
#define INFINITESIMAL_DOUBLE_2  0.1
#define SAVE_IMG_INTERVAL -1 // 0 to not save at all. -1 to save last frame only.
#define VIS_INTERVAL 100

//environ
//{left bottom coordinates x, coordinates x, right top x, y}
#define BLANK 5
#define COORD_TYPE double

#define PI 3.1415926535897931

// colors
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

template<typename T>
CvPoint cv_plot_coord(T x, T y) {
  return cvPoint(PLOT_SCALE * x, PLOT_SCALE * y);
}

template<typename T>
CvPoint cv_plot_coord(T p) {
  return cvPoint(PLOT_SCALE * p.x, PLOT_SCALE * p.y);
}

#endif