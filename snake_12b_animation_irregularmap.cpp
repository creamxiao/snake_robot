///////////////////////////////////
////////////    Xiaolong Wang
////////////    2018/5/9
///////////////////////////////////

#include <iostream>
#include <fstream> // for output files
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <math.h>
#include <algorithm>
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <thread>
#include <typeinfo>
#include <ctime>
#include <cstdlib>
#include <regex>

// Create a model by column
// #include "ClpSimplex.hpp"
// #include "CoinHelperFunctions.hpp"
// #include "CoinTime.hpp"
// #include "CoinBuild.hpp"
// #include "CoinModel.hpp"
#include <iomanip>
#include <cassert>

#include <qpOASES.hpp> // quadratic programming

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#ifndef _DOSL_ALGORITHM // can pass at command line during compilation: -D_DOSL_ALGORITHM=AStar
    #define _DOSL_ALGORITHM  AStar
#endif

#ifndef _OS_FLAG
    #define _OS_FLAG  1 // 1 = Linux, 2 = Mac
#endif

#include <dosl/dosl>
// Local libraries/headers
#include <dosl/aux-utils/cvParseMap2d.hpp>
#include <dosl/aux-utils/double_utils.hpp>
#include <dosl/aux-utils/string_utils.hpp> // compute_program_path

//#include "local-include/cvParseMap2d.h"
#include "local-include/RSJparser.tcc"

using namespace std;
using namespace cv;
USING_NAMESPACE_QPOASES
//using namespace arma;

#define RRT 1 //switch of rrt algorithm, 1 = RRT, 0 = A star

#define map_selection 1 //choose map

#define C1 1.0 // coefficient of lenth term in the cost function
#define C2 2.0 // coefficient of torque term in the cost function
#define C3 2.0 // coefficient of touching wall term in the cost function
#define C4 0.5 // coefficient of heuristic function

#if map_selection == 1
  #define OBSTHRESHOLD 100
  #define PLOT_SCALE 3
  #define VERTEX_SIZE 1
  #define LINE_THICKNESS 1 // CV_FILLED
  #define A 8.0
  #define L 8.25
  #define EL 0.5 // Discreet expansion increasement
#else
  #define PLOT_SCALE 5
  #define VERTEX_SIZE 1
  #define LINE_THICKNESS 1 // CV_FILLED
  #define A 2.5
  #define L 2.5858
  #define A_H 3.25
  #define A_L 1.7929
#endif

// physic/mechanic parameters
#define MAXFORCE 750
#define DRILLFORCE 250
#define MU 0.2 // friction coefficient
#define LINKMASS 0.5 // kg
#define HEADMASS 1 // kg
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
//426x240
/*
int approx_floor (double x, double tol = INFINITESIMAL_DOUBLE) {
    int ret = floor (x);
    if (ret + 1 - x < tol) ++ret;
    return (ret);
}
*/
class POINT : public AStar::Node<POINT,double>{
public:
	double x, y;
	POINT() {	};
	POINT(double x, double y)
	{
		this->x = x;
		this->y = y;
	};

	bool operator == (const POINT & n) const {
			//if (x == n.x && y == n.y) return (true);
			return (fabs(x - n.x) < INFINITESIMAL_DOUBLE && fabs(y - n.y) < INFINITESIMAL_DOUBLE);
	}

	bool operator != (const POINT & n) const {

			//if (x != n.x || y != n.y) return (true);
			return (fabs(x - n.x) >= INFINITESIMAL_DOUBLE || fabs(y - n.y) >= INFINITESIMAL_DOUBLE);
	}
  // Inherited functions being overwritten
  int getHashBin (void) const {
      return (abs(x));
  }
};

class PointHash
{
public:
	size_t operator()(const POINT& p) const
	{
		return fabs(p.x);
		//return p.x * 1000 + p.y;
	}
};

class PointEqual
{
public:
	bool operator()(const POINT & p1, const POINT & p2) const
	{
		if (p1.x == p2.x && p1.y == p2.y) return true;
		return false;
	}
};

// Virtual Shortest Path, for cost function and heuristic function
vector<POINT> VSP;

template<typename T>
CvPoint cv_plot_coord_out(T x, T y) {
  return( cvPoint( round(PLOT_SCALE * x), round(PLOT_SCALE * y) ) );
}

class LINK : public AStar::Node<LINK, double>
{
public:
  double x, y;
	double theta; //orientation, radian
	double expansion; // 0 - 1
  bool head = false;
  bool brace = false;
  bool continue_expansion = true;
  int iClosestP = -1; // the index of closest point in the VSP to the end of this LINK
  vector<vector<double> > forces;//Fx, Fy, x, y, scalar, weight
  double gAccX = 0, gAccY = 0; // accumulated gravity on this link
  double gAccT = 0; // accumulated gravity-torque about the end of this link

	LINK() {
  }
	LINK(double x, double y, double theta, double expansion)
	{
		this->x = x;
		this->y = y;
		this->theta = theta;
		this->expansion = expansion;
	}

  void reverseLINK(){
    double ox = this->x, oy = this->y, otheta = this->theta;
    this->x = ox + (2 * A + L * this->expansion) * cos(otheta);
    this->y = oy + (2 * A + L * this->expansion) * sin(otheta);
    this->theta = otheta - PI;
  }

	int getHashBin (void) const {
		return MAX(round(fabs(x) + INFINITESIMAL_DOUBLE), round(fabs(y) - INFINITESIMAL_DOUBLE));
	}

	bool operator == (const LINK & n) const { // theta, 2degree
    if(this->forces.size() != n.forces.size()) return false;
    for (int i = 0; i < this->forces.size(); i++){
      if(this->forces[i][0] != n.forces[i][0] || this->forces[i][1] != n.forces[i][1]) return false; // only care about direction of forces, not position
    }
		return (fabs(this->x - n.x) < INFINITESIMAL_DOUBLE_2 && fabs(this->y - n.y) < INFINITESIMAL_DOUBLE_2
       && (fabs(this->theta - n.theta) < (INFINITESIMAL_DOUBLE_2 / 10) || fabs(this->theta + 2 * PI - n.theta) < (INFINITESIMAL_DOUBLE_2 / 10) || fabs(this->theta - 2 * PI - n.theta) < (INFINITESIMAL_DOUBLE_2 / 10))
       && fabs(this->expansion - n.expansion) < INFINITESIMAL_DOUBLE_2);
	}

	bool operator != (const LINK & n) const {
    if(this->forces.size() != n.forces.size()) return true;
    for (int i = 0; i < this->forces.size(); i++){
      if(this->forces[i] != n.forces[i]) return true;
    }
    return (fabs(x - n.x) >= INFINITESIMAL_DOUBLE_2 || fabs(y - n.y) >= INFINITESIMAL_DOUBLE_2 || fabs(theta - n.theta) >= (INFINITESIMAL_DOUBLE_2 / 10) || fabs(expansion - n.expansion) >= INFINITESIMAL_DOUBLE_2);
	}

	bool reachGoal(const LINK & n) {
		//check the other end of the link
    if (fabs(this->theta - n.theta) > INFINITESIMAL_DOUBLE_2) return false;
    for(double i = 0.0; i <= (A * 2 + this->expansion * L); i+=0.4){
      if(sqrt(pow(this->x + i * cos(this->theta) - n.x, 2) + pow(this->y + i * sin(this->theta)- n.y, 2)) <= 1) {
        //printf("head theta: %.3f PI, goal theta: %.3f PI, fabs(this->theta - n.theta) = %g\n", this->theta / PI, n.theta / PI, fabs(this->theta - n.theta));
        return true;
      }
    }
    return false;

		//return (sqrt(pow(x + (expansion * L + A * 2) * cos(theta) - n.x, 2) + pow(y + (expansion * L + A * 2) * sin(theta)- n.y, 2)) <= 1);
	}

	POINT getPoint() const{
		return POINT(this->x, this->y);
	}

  POINT getHead() const{
    return POINT(this->x + (2 * A + L * this->expansion) * cos(this->theta), this->y + (2 * A + L * this->expansion) * sin(this->theta));
  }

  double getLength() const{
    return (A * 2 + L * this->expansion);
  }

  double getHeadX() const{
    return (this->x + (2 * A + L * this->expansion) * cos(this->theta));
  }

  double getHeadY() const{
    return (this->y + (2 * A + L * this->expansion) * sin(this->theta));
  }

  void makeHead(){
		this->head = true;
    this->expansion = 0;

	}

	void plotLINK(Mat & plot_canvas){
		//first half of link red
		line(plot_canvas, cv_plot_coord_out(this->x, this->y),
				cv_plot_coord_out((this->x + A * cos(this->theta)), (this->y + A * sin(this->theta))), CV_RGB(255, (this->brace? 0 : 77), (this->brace? 0 : 77)), LINE_THICKNESS * PLOT_SCALE, CV_AA);

		if (this->expansion > 0){
			//expansion blue
			line(plot_canvas, cv_plot_coord_out((this->x + A * cos(this->theta)), (this->y + A * sin(this->theta))),
					cv_plot_coord_out((this->x + (A + this->expansion * L) * cos(this->theta)), (this->y + (A + this->expansion * L) * sin(this->theta))), CV_RGB((this->brace? 0 : 77), (this->brace? 0 : 77), 255), LINE_THICKNESS * PLOT_SCALE, CV_AA);
		}
		//second half of link green, if head, black
    if (this->head){
      line(plot_canvas, cv_plot_coord_out(this->x + (A + this->expansion * L) * cos(this->theta), this->y + (A + this->expansion * L) * sin(this->theta)),
  				cv_plot_coord_out(this->x + (2 * A + this->expansion * L) * cos(this->theta), this->y + (2 * A + this->expansion * L) * sin(this->theta)), CV_RGB(0, 0, 0), LINE_THICKNESS * PLOT_SCALE * 2 / 3, CV_AA);
      circle(plot_canvas, cv_plot_coord_out(this->x + (1.5 * A + this->expansion * L) * cos(this->theta), this->y + + (1.5 * A + this->expansion * L) * sin(this->theta)),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(0, 0, 0), -1, CV_AA);
    }
    else{
  		line(plot_canvas, cv_plot_coord_out((this->x + (A + this->expansion * L) * cos(this->theta)), (this->y + (A + this->expansion * L) * sin(this->theta))),
  				cv_plot_coord_out((this->x + (2 * A + this->expansion * L) * cos(this->theta)), (this->y + (2 * A + this->expansion * L) * sin(this->theta))), CV_RGB((this->brace? 0 : 77), 255, (this->brace? 0 : 77)), LINE_THICKNESS * PLOT_SCALE, CV_AA);
    }
		//origin of link
    circle(plot_canvas, cv_plot_coord_out(this->x, this->y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(50, 50, 50), -1, CV_AA);
		//circle(plot_canvas, cv_plot_coord_out((this->x + (2 * A + this->expansion * L) * cos(this->theta)), (this->y + (2 * A + this->expansion * L) * sin(this->theta))),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(50, 50, 50), -1, 8);
  }

  void getGraAcc(double mass, double parentAccFX = 0, double parentAccFY = 0, double parentAccT = 0){ // the torque is about the end of this LINK
    GRAVITYDIR;
    gAccX = parentAccFX + g[0] * mass;
    gAccY = parentAccFY + g[1] * mass;
    gAccT = parentAccT
            + ((2 * A + L * expansion) * cos(theta + PI) * parentAccFY
            - (2 * A + L * expansion) * sin(theta + PI) * parentAccFX
            + (2 * A + L * expansion) / 2 * cos(theta + PI) * g[1] * mass
            - (2 * A + L * expansion) / 2 * sin(theta + PI) * g[0] * mass) * P2M_SCALE;
  }
};

class LINKHash
{
public:
	size_t operator()(const LINK & p) const
	{
		return MAX(round(fabs(p.x) + INFINITESIMAL_DOUBLE), round(fabs(p.y) - INFINITESIMAL_DOUBLE));
	}
};

class LINKEqual
{
public:
	bool operator()(const LINK & p1, const LINK & p2) const
	{
		return (fabs(p1.x - p2.x) < INFINITESIMAL_DOUBLE_2 && fabs(p1.y - p2.y) < INFINITESIMAL_DOUBLE_2 && fabs(sin(p1.theta) - sin(p2.theta)) < INFINITESIMAL_DOUBLE_2 && fabs(p1.expansion - p2.expansion) < INFINITESIMAL_DOUBLE_2);
	}
};

LINK nextPos(LINK cur, LINK pre){// to get next position for the animation
  double angle = atan2(pre.y - cur.y, pre.x - cur.x); // from curent to previous link
  LINK next = cur;
  printf("angle = %g PI, pre.theta = %g PI\n", angle / PI, pre.theta / PI);
  if (fabs(angle - pre.theta) > PI){
    if (fabs(angle + 2 * PI - pre.theta) > PI){
      angle -= 2 * PI;
    }else{
      angle += 2 * PI;
    }
  }
  if (fabs(angle - pre.theta) > (PI / 4)){
    angle = ((angle - pre.theta) > 0? 1 : -1) * PI / 4 + pre.theta;
    if (angle > PI){
      angle -= 2 * PI;
    }else if (angle < - PI){
      angle += 2 * PI;
    }

    printf("corrected angle = %g PI, pre.theta = %g PI\n", angle / PI, pre.theta / PI);
  }

  next.x = pre.x + (A * 2 + L * cur.expansion) * cos(angle - PI);
  next.y = pre.y + (A * 2 + L * cur.expansion) * sin(angle - PI);
  next.theta = angle;
  return next;
}

LINK nextPosR(LINK cur, LINK ne){// to get next position for the animation
  double angle = atan2(cur.getHeadY() - ne.getHeadY(), cur.getHeadY() - ne.getHeadY()); // from curent to previous link
  LINK next = cur;
  printf("angle = %g PI, ne.theta = %g PI\n", angle / PI, ne.theta / PI);
  if (fabs(angle - ne.theta) > PI){
    if (fabs(angle + 2 * PI - ne.theta) > PI){
      angle -= 2 * PI;
    }else{
      angle += 2 * PI;
    }
  }
  if (fabs(angle - ne.theta) > (PI / 4)){
    angle = ((angle - ne.theta) > 0? 1 : -1) * PI / 4 + ne.theta;
    if (angle > PI){
      angle -= 2 * PI;
    }else if (angle < - PI){
      angle += 2 * PI;
    }

    printf("corrected angle = %g PI, ne.theta = %g PI\n", angle / PI, ne.theta / PI);
  }

  next.x = ne.getHeadX();
  next.y = ne.getHeadY();
  next.theta = angle;
  return next;
}

POINT triCorner(POINT p1, POINT p2, double x, double y, double z){
  double n = (pow(z, 2) - pow(y, 2) + pow(x, 2)) / (2 * z);
  double m = sqrt(pow(x, 2) - pow(n, 2));
  return POINT(p2.x + (p1.x - p2.x) * (z - n) / z + (p1.y - p2.y) / z * m, p2.y + (p1.y - p2.y) * (z - n) / z + (p2.x - p1.x) / z * m);
}

vector<LINK> getConfig(int part){
  vector<LINK> path;
  if (part == 1){ //  tapping

    path.push_back(LINK(181, 88, 2.35619, 0.5));
    path.push_back(LINK(166.769, 102.231, 3.14159, 0));
    path.push_back(LINK(150.769, 102.231, 3.92699, 0));
    path.push_back(LINK(139.456, 90.9168, 4.71239, 0));
    path.push_back(LINK(139.456, 74.9168, 5.49779, 0.25));
    path.push_back(LINK(152.228, 62.1447, 6.28319, 0));
    path.push_back(LINK(168.228, 62.1447, 5.49779, 0.25));
    path.push_back(LINK(181, 49.3726, 4.71239, 0));
    path.push_back(LINK(181, 33.3726, 3.92699, 0));
    path.push_back(LINK(169.686, 22.0589, 3.14159, 0));
    path.push_back(LINK(153.686, 22.0589, 2.35619, 0));
    path.push_back(LINK(142.373, 33.3726, 1.5708, 0));
    path.push_back(LINK(142.373, 49.3726, 2.35619, 0.25));
    path.push_back(LINK(129.6, 62.1447, 3.14159, 0));
    path.push_back(LINK(113.6, 62.1447, 3.53429, 1));
    path.push_back(LINK(91.1964, 52.8646, 3.14159, 0));
    path.push_back(LINK(75.1964, 52.8646, 3.53429, 0.25));
    path.push_back(LINK(58.5088, 45.9524, 3.53429, 0));
    path.push_back(LINK(43.7267, 39.8295, 3.14159, 0.5));
/*
    path.push_back(LINK(41, 29, 8.63938, 0));
    path.push_back(LINK(29.6863, 40.3137, 7.85398, 0));
    path.push_back(LINK(29.6863, 56.3137, 7.06858, 0));
    path.push_back(LINK(41, 67.6274, 6.28319, 0.5));
    path.push_back(LINK(61.125, 67.6274, 7.06858, 0));
    path.push_back(LINK(72.4387, 78.9411, 7.85398, 0));
    path.push_back(LINK(72.4387, 94.9411, 8.63938, 0));
    path.push_back(LINK(61.125, 106.255, 9.42478, 0.5));
    path.push_back(LINK(41, 106.255, 8.63938, 0));
    path.push_back(LINK(29.6863, 117.569, 7.85398, 0));
    path.push_back(LINK(29.6863, 133.569, 7.85398, 1));
    path.push_back(LINK(29.6863, 157.819, 7.46128, 0));
    path.push_back(LINK(35.8092, 172.601, 7.46128, 0.25));
    path.push_back(LINK(42.7214, 189.288, 7.46128, 0.25));
    path.push_back(LINK(49.6337, 205.976, 7.06858, 0));
    path.push_back(LINK(60.9474, 217.289, 6.28319, 0));

    path.push_back(LINK(139, 88, 6.28319, 0));
    path.push_back(LINK(155, 88, 5.89049, 0));
    path.push_back(LINK(169.782, 81.8771, 5.49779, 0));
    path.push_back(LINK(181.096, 70.5634, 4.71239, 0.5));
    path.push_back(LINK(181.096, 50.4384, 4.31969, 0));
    path.push_back(LINK(174.973, 35.6563, 3.53429, 0));
    path.push_back(LINK(160.191, 29.5333, 2.74889, 0));
    path.push_back(LINK(145.409, 35.6563, 1.9635, 0));
    path.push_back(LINK(139.286, 50.4384, 2.35619, 0));
    path.push_back(LINK(127.972, 61.7521, 3.14159, 0));
    path.push_back(LINK(111.972, 61.7521, 3.53429, 1));
    path.push_back(LINK(89.568, 52.472, 3.14159, 0.25));
    path.push_back(LINK(71.5055, 52.472, 3.53429, 0));
    path.push_back(LINK(56.7234, 46.3491, 3.53429, 0));
    path.push_back(LINK(41.9413, 40.2261, 3.14159, 0.25));*/
  }else if (part == 2){ // new tapping
    path.push_back(LINK(183.917, 85.0832, 2.35619, 0.25));
    path.push_back(LINK(171.145, 97.8553, 3.14159, 0.5));
    path.push_back(LINK(151.02, 97.8553, 3.92699, 0));
    path.push_back(LINK(139.706, 86.5416, 4.71239, 0));
    path.push_back(LINK(139.706, 70.5416, 5.49779, 0));
    path.push_back(LINK(151.02, 59.2279, 6.28319, 0.25));
    path.push_back(LINK(169.082, 59.2279, 5.49779, 0));
    path.push_back(LINK(180.396, 47.9142, 4.71239, 0));
    path.push_back(LINK(180.396, 31.9142, 3.92699, 0));
    path.push_back(LINK(169.082, 20.6005, 3.14159, 0));
    path.push_back(LINK(153.082, 20.6005, 2.35619, 0));
    path.push_back(LINK(141.768, 31.9142, 1.5708, 0.25));
    path.push_back(LINK(141.768, 49.9767, 2.35619, 0));
    path.push_back(LINK(130.455, 61.2904, 3.14159, 0));
    path.push_back(LINK(114.455, 61.2904, 3.53429, 1));
    path.push_back(LINK(92.0507, 52.0103, 3.53429, 0));
    path.push_back(LINK(77.2686, 45.8874, 3.53429, 0));
    path.push_back(LINK(62.4866, 39.7644, 3.14159, 0.625));
    path.push_back(LINK(41.3303, 39.7644, 3.14159, 0.125));
    /*
    path.push_back(LINK(46.8336, 23.1664, 8.63938, 0));
    path.push_back(LINK(35.5199, 34.4801, 8.24668, 0));
    path.push_back(LINK(29.397, 49.2622, 7.85398, 0));
    path.push_back(LINK(29.397, 65.2622, 7.06858, 0));
    path.push_back(LINK(40.7107, 76.5759, 6.28319, 0.5));
    path.push_back(LINK(60.8357, 76.5759, 7.06858, 0));
    path.push_back(LINK(72.1494, 87.8896, 7.85398, 0));
    path.push_back(LINK(72.1494, 103.89, 8.63938, 0));
    path.push_back(LINK(60.8357, 115.203, 9.42478, 0.5));
    path.push_back(LINK(40.7107, 115.203, 8.63938, 0));
    path.push_back(LINK(29.397, 126.517, 7.85398, 0));
    path.push_back(LINK(29.397, 142.517, 7.46128, 1));
    path.push_back(LINK(38.6771, 164.921, 7.46128, 1));
    path.push_back(LINK(47.9571, 187.325, 7.85398, 0.25));
    path.push_back(LINK(47.9571, 205.388, 7.06858, 0.25));
    path.push_back(LINK(60.7293, 218.16, 6.28319, 0));

    path.push_back(LINK(131, 88, 6.28319, 0));
    path.push_back(LINK(147, 88, 6.28319, 0.5));
    path.push_back(LINK(167.125, 88, 5.49779, 0.5));
    path.push_back(LINK(181.356, 73.7695, 4.71239, 0.5));
    path.push_back(LINK(181.356, 53.6445, 3.92699, 0.5));
    path.push_back(LINK(167.125, 39.414, 3.14159, 0));
    path.push_back(LINK(151.125, 39.414, 2.35619, 0));
    path.push_back(LINK(139.811, 50.7277, 2.35619, 0));
    path.push_back(LINK(128.498, 62.0414, 3.14159, 0));
    path.push_back(LINK(112.498, 62.0414, 3.53429, 1));
    path.push_back(LINK(90.0935, 52.7613, 3.14159, 0));
    path.push_back(LINK(74.0935, 52.7613, 3.53429, 0.25));
    path.push_back(LINK(57.4059, 45.8491, 3.53429, 0));
    path.push_back(LINK(42.6239, 39.7261, 3.14159, 0.25));*/
  }

  reverse(path.begin(), path.end());
  for (auto & i : path){
    i.reverseLINK();
  }

  return path;
}

double dist(POINT a, POINT b){
  return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
}
int MAX_X, MAX_Y;

bool withinFig(int y, int x){
  return x >= 0 && x < MAX_X && y >= 0 && y < MAX_Y;
}


int main(int argc, char *argv[])
{
	string program_fName (argv[0]);
	string program_folderName = program_fName.substr(0, program_fName.find_last_of("/\\")+1);
	string expt_f_name = program_folderName + "exptfiles/experiments_12b_animation_irregularMap.json", expt_name = "path_plan_1";

	if (argc > 1) {
		expt_f_name = argv[1];
		expt_name = argv[2];
	}

  // Read from file
  ifstream my_fstream (expt_f_name);
  RSJresource expt_container = RSJresource (my_fstream)[expt_name];
  LINK myGoalLink(expt_container["goal"][0].as<int>(), expt_container["goal"][1].as<int>(), expt_container["angle_g"].as<double>() * PI, 0);
  LINK myStartLink(expt_container["start"][0].as<int>(), expt_container["start"][1].as<int>(), expt_container["angle_s"].as<double>() * PI, 0);

  // Read map from file
  string expt_folderName = expt_f_name.substr(0, expt_f_name.find_last_of("/\\")+1);

  string map_image_fName = expt_folderName + expt_container["map_name"].as<std::string>();
  Mat original_map = imread(map_image_fName, CV_LOAD_IMAGE_COLOR); // second parameter computes representative points

  Mat grey_map, realtime_display;
  cvtColor(original_map, grey_map, CV_RGB2GRAY);
  threshold(grey_map, grey_map, 20, 255, 0);
  cvtColor(grey_map, realtime_display, CV_GRAY2RGB);
  resize(realtime_display, realtime_display, Size(), PLOT_SCALE, PLOT_SCALE, INTER_NEAREST);

  MAX_X = realtime_display.cols;
  MAX_Y = realtime_display.rows;

	vector<vector<LINK> > allLks;
  allLks.push_back(getConfig(1));
  allLks.push_back(getConfig(2));


  // ===========================================================================
  //hereafter shows the final configuration
	Mat start_canvas = realtime_display.clone();

	//draw final configuration
	for(auto b : allLks[0]){
		line(start_canvas, cv_plot_coord_out(b.x, b.y),
				cv_plot_coord_out((b.x + (2 * A + b.expansion * L) * cos(b.theta)), (b.y + (2 * A + b.expansion * L) * sin(b.theta))), CV_RGB(153, 204, 255), LINE_THICKNESS * PLOT_SCALE);
	}

  circle(start_canvas, cv_plot_coord_out(myStartLink.x, myStartLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);
  circle(start_canvas, cv_plot_coord_out(myGoalLink.x, myGoalLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);

  char imgFname[1024];
  time_t now = time(0);
  tm* localtm = localtime(&now);
	sprintf(imgFname, "outfiles/animation/%s_%d_%d_%d_%d_%d_begin_%s.png", argv[0], 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, localtm->tm_hour, localtm->tm_min, _OS_FLAG == 1? "Linux" : "Mac");
	// imwrite(regex_replace(regex_replace(imgFname, regex("\n"), ""), regex(":"), "_"), start_canvas);
  //imwrite(imgFname, start_canvas);
  cout << "final configuration shown" << endl;
	imshow("Feed in animation", start_canvas);
	cvWaitKey();
	start_canvas.release();

  //animation starts hereafter =============================================================================

	vector<LINK> robot, alt_Lks = allLks[0];

  // insert a external link as new fed-in
	alt_Lks.insert(alt_Lks.begin(), LINK(alt_Lks.front().x - 2 * A * cos(alt_Lks.front().theta), alt_Lks.front().y - 2 * A * sin(alt_Lks.front().theta), alt_Lks.front().theta, 0));


  double endFeedSpeed = 5.0, drillFeedSpeed = 1.0;
  double dt = 0.04; // time interval
  double t_per_LINK = (2 * A) / endFeedSpeed;
  double steps_per_LINK = t_per_LINK / dt;
  bool arrived = false, swi;
  int cur_po = 0; //current position (LINK No. in the alter_path)
  double x_ret, y_ret, theta_ret, expansion_ret;
  int change_po = cur_po - 1;
  int tar_path = 0; // target path No. in allLks
	POINT HEAD; // the head of the robot
  int T = 0; // time
  vector<POINT> dists;
  int shift = 7; // when to contract links

	for(; !arrived; T++){
		//reset canvas
		start_canvas = realtime_display.clone();

		//shortcuts
		for(int b = 0; b < (allLks[0].size() - 1); b++){
			line(start_canvas, cv_plot_coord_out(allLks[0][b].x, allLks[0][b].y),
					cv_plot_coord_out(allLks[0][b + 1].getHeadX(), allLks[0][b + 1].getHeadY()), CV_RGB(255, 153, 51), LINE_THICKNESS, CV_AA);
		}

    //draw auxilary configuration in light green color
		for(auto b : alt_Lks){
			line(start_canvas, cv_plot_coord_out(b.x, b.y),
					cv_plot_coord_out((b.x + (2 * A + b.expansion * L) * cos(b.theta)), (b.y + (2 * A + b.expansion * L) * sin(b.theta))), CV_RGB(153, 255, 204), LINE_THICKNESS * PLOT_SCALE, CV_AA);
		}

		//draw final configuration in dark color
		for(auto b : allLks[0]){
			line(start_canvas, cv_plot_coord_out(b.x, b.y),
					cv_plot_coord_out((b.x + (2 * A + b.expansion * L) * cos(b.theta)), (b.y + (2 * A + b.expansion * L) * sin(b.theta))), CV_RGB(153, 204, 255), LINE_THICKNESS * PLOT_SCALE, CV_AA);
		}

		//move the robot
		if(T != 0){
      if (change_po != cur_po){
        x_ret = alt_Lks[cur_po].getHeadX() - HEAD.x;
        y_ret = alt_Lks[cur_po].getHeadY() - HEAD.y;
        if (dists.size() != robot.size()) dists.resize(robot.size());
        for(int it = 0; it < robot.size(); it++){
          dists[it].x = alt_Lks[cur_po - it].x - robot[it].x;
          dists[it].y = alt_Lks[cur_po - it].y - robot[it].y;
          printf("dists[%d] = (%g, %g)\n", it, dists[it].x, dists[it].y);
        }
        change_po = cur_po;
      }
			//update the head of robot
      HEAD.x += (x_ret / steps_per_LINK);
      HEAD.y += (y_ret / steps_per_LINK);
      //update position of LINKs of robot
			for(int it = 0; it < robot.size(); it++){
				robot[it].x += (dists[it].x / steps_per_LINK);
				robot[it].y += (dists[it].y / steps_per_LINK);
				if (it > 0){
					robot[it].theta = atan2(robot[it - 1].y - robot[it].y, robot[it - 1].x - robot[it].x);
          robot[it].expansion += ((alt_Lks[cur_po - it].expansion - alt_Lks[cur_po - 1 - it].expansion) / steps_per_LINK);
  			}else{
					robot[it].theta = atan2(HEAD.y - robot[it].y, HEAD.x - robot[it].x);
          robot[it].expansion = (sqrt(pow(HEAD.x - robot[it].x, 2) + pow(HEAD.y - robot[it].y, 2)) - 2 * A) / L;
        }
			}

		}else{
			HEAD = alt_Lks[1].getPoint();
		}
    //add new LINKs
		if(robot.empty() || ((robot.back().x - myStartLink.x) * cos(myStartLink.theta) + (robot.back().y - myStartLink.y) * sin(myStartLink.theta)) >= 0.0){
			robot.push_back(alt_Lks.front());
      printf("new-insert LINK: coordinates (%g, %g), theta: %g PI\n", alt_Lks.front().x, alt_Lks.front().y, alt_Lks.front().theta / PI);
			cur_po++;
      if(robot.size() == 1)
        robot.front().makeHead();
		}

    printf("Animation time: %g sec\n", (double)T * dt);


    // plot robot links
    for (int i = 0; i < robot.size(); i++){
      robot[i].plotLINK(start_canvas);
    }

    //draw the broken line to the wall as the robot approaches
    swi = true;
    for (int i = 0; realtime_display.at<Vec3b>((int)round(robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE,
                                                (int)round(robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE)[0] > OBSTHRESHOLD; i++){
      if (swi){
        line(start_canvas, cv_plot_coord_out(robot.front().getHeadX() + i * cos(robot.front().theta), robot.front().getHeadY() + i * sin(robot.front().theta)),
            cv_plot_coord_out(robot.front().getHeadX() + (i + 1) * cos(robot.front().theta), robot.front().getHeadY() + (i + 1) * sin(robot.front().theta)), CV_RGB(100, 100, 100), LINE_THICKNESS, CV_AA);
      }
      swi = !swi;
    }

    circle(start_canvas, cv_plot_coord_out(myStartLink.x, myStartLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);
    circle(start_canvas, cv_plot_coord_out(myGoalLink.x, myGoalLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);

    sprintf(imgFname, "Time = %.2f sec", (double)T * dt);
    putText(start_canvas, imgFname, cvPoint(45, 345), 1, 1, CV_RGB(0, 0, 0), 1, 8, false);
    //cvPoint(29, 733)
    //cvPoint(45, 345)
		sprintf(imgFname, "outfiles/animation/link_path_%d_%d_%d_screenshot_%d.png", 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, T);
		// imwrite(imgFname, start_canvas);
		imshow("Feed in animation", start_canvas);
		cvWaitKey(1);
		start_canvas.release();
		if (robot.size() == alt_Lks.size()){
      arrived = true;
    }
	}
  cout << "Tapping config reached!" << endl;

  myGoalLink.x += (1.0 - alt_Lks.back().expansion) * L * cos(robot.front().theta);
  myGoalLink.y += (1.0 - alt_Lks.back().expansion) * L * sin(robot.front().theta);

  // drill the first hole ==================================================
  for(; robot.front().expansion < 1; T++){
    if ((1 - robot.front().expansion) > drillFeedSpeed * dt){
      //update position of LINKs of robot
      robot.front().expansion += drillFeedSpeed * dt / L;
      //update the head of robot
      HEAD.x += drillFeedSpeed * dt * cos(robot.front().theta);
      HEAD.y += drillFeedSpeed * dt * sin(robot.front().theta);
    }else{
      robot.front().expansion = 1.0;
      HEAD.x = robot.front().getHeadX();
      HEAD.y = robot.front().getHeadY();
    }
    cout << "Distance remaining: " << 1 - robot.front().expansion << endl;
    // plot the drilled hole
    line(realtime_display, cv_plot_coord_out(robot.front().x, robot.front().y),
				cv_plot_coord_out((robot.front().x + (1.8 * A + robot.front().expansion * L) * cos(robot.front().theta)), (robot.front().y + (1.8 * A + robot.front().expansion * L) * sin(robot.front().theta))), CV_RGB(255, 255, 255), 2 * PLOT_SCALE);

    start_canvas = realtime_display.clone();

    // plot robot links
    for (int i = 0; i < robot.size(); i++){
      robot[i].plotLINK(start_canvas);
    }

    circle(start_canvas, cv_plot_coord_out(myStartLink.x, myStartLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);
    circle(start_canvas, cv_plot_coord_out(myGoalLink.x, myGoalLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);

    //draw the broken line to the wall as the robot approaches
    swi = true;
    for (int i = 0; realtime_display.at<Vec3b>((int)round(robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE,
                                            (int)round(robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE)[0] > OBSTHRESHOLD; i++){
      if (swi){
        line(start_canvas, cv_plot_coord_out(robot.front().getHeadX() + i * cos(robot.front().theta), robot.front().getHeadY() + i * sin(robot.front().theta)),
            cv_plot_coord_out(robot.front().getHeadX() + (i + 1) * cos(robot.front().theta), robot.front().getHeadY() + (i + 1) * sin(robot.front().theta)), CV_RGB(100, 100, 100), LINE_THICKNESS, CV_AA);
      }
      swi = !swi;
    }

    sprintf(imgFname, "Time = %.2f sec", (double)T * dt);
    putText(start_canvas, imgFname, cvPoint(45, 345), 1, 1, CV_RGB(0, 0, 0), 1, 8, false);

		sprintf(imgFname, "outfiles/animation/link_path_%d_%d_%d_screenshot_%d.png", 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, T);
		// imwrite(imgFname, start_canvas);
		imshow("Feed in animation", start_canvas);
		cvWaitKey(1);
  }

// transfrom ========================================

  alt_Lks = allLks[1];
  // insert a external link as new fed-in
  alt_Lks.insert(alt_Lks.begin(), LINK(alt_Lks.front().x - 2 * A * cos(alt_Lks.front().theta), alt_Lks.front().y - 2 * A * sin(alt_Lks.front().theta), alt_Lks.front().theta, 0));

  dists.resize(robot.size());

  for(int it = 0; it < dists.size(); it++){
    if (it < alt_Lks.size()){
      dists[it].x = alt_Lks.rbegin()[it].x - robot[it].x;
      dists[it].y = alt_Lks.rbegin()[it].y - robot[it].y;
    }else{
      dists[it].x = alt_Lks.front().x - cos(alt_Lks.front().theta) * (2 * A + L) * (it - alt_Lks.size() + 1) - robot[it].x;
      dists[it].y = alt_Lks.front().y - sin(alt_Lks.front().theta) * (2 * A + L) * (it - alt_Lks.size() + 1) - robot[it].y;
    }
  }

  for(; robot.front().getPoint() != alt_Lks.back().getPoint(); T++){
    // the rest of the robot approaches
    cout << "Approaching" << endl;
    //update position of LINKs of robot
    for(int it = 0; it < robot.size(); it++){
      robot[it].x += (dists[it].x / steps_per_LINK);
      robot[it].y += (dists[it].y / steps_per_LINK);

      if (it > 0){
        robot[it].theta = atan2(robot[it - 1].y - robot[it].y, robot[it - 1].x - robot[it].x);
        robot[it].expansion = (sqrt(pow(robot[it - 1].x - robot[it].x, 2) + pow(robot[it - 1].y - robot[it].y, 2)) - 2 * A) / L;
      }else{
        robot[it].theta = atan2(HEAD.y - robot[it].y, HEAD.x - robot[it].x);
        robot[it].expansion = (sqrt(pow(HEAD.x - robot[it].x, 2) + pow(HEAD.y - robot[it].y, 2)) - 2 * A) / L;
      }
    }

    //reset canvas
    start_canvas = realtime_display.clone();

    //draw final configuration in light blue color
    for(auto b : alt_Lks){
      line(start_canvas, cv_plot_coord_out(b.x, b.y),
          cv_plot_coord_out((b.x + (2 * A + b.expansion * L) * cos(b.theta)), (b.y + (2 * A + b.expansion * L) * sin(b.theta))), CV_RGB(153, 204, 255), LINE_THICKNESS * PLOT_SCALE, CV_AA);
    }

    printf("Animation time: %g sec\n", (double)T * dt);

    if(((robot.back().x - myStartLink.x) * cos(myStartLink.theta) + (robot.back().y - myStartLink.y) * sin(myStartLink.theta)) >= 0.0){
			robot.push_back(alt_Lks.front());
      printf("new-insert LINK: coordinates (%g, %g), theta: %g PI\n", alt_Lks.front().x, alt_Lks.front().y, alt_Lks.front().theta / PI);
		}else if (((robot.rbegin()[1].x - myStartLink.x) * cos(myStartLink.theta) + (robot.rbegin()[1].y - myStartLink.y) * sin(myStartLink.theta)) < 0.0){
      robot.pop_back();
      printf("pop-out LINK\n");
    }

    /// plot robot links
    for (int i = 0; i < robot.size(); i++){
      robot[i].plotLINK(start_canvas);
    }

    //draw the broken line to the wall as the robot approaches
    swi = true;
    for (int i = 0; realtime_display.at<Vec3b>((int)round(robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE,
                                            (int)round(robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE)[0] > OBSTHRESHOLD; i++){
      if (swi){
        line(start_canvas, cv_plot_coord_out(robot.front().getHeadX() + i * cos(robot.front().theta), robot.front().getHeadY() + i * sin(robot.front().theta)),
            cv_plot_coord_out(robot.front().getHeadX() + (i + 1) * cos(robot.front().theta), robot.front().getHeadY() + (i + 1) * sin(robot.front().theta)), CV_RGB(100, 100, 100), LINE_THICKNESS, CV_AA);
      }
      swi = !swi;
    }

    circle(start_canvas, cv_plot_coord_out(myStartLink.x, myStartLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);
    circle(start_canvas, cv_plot_coord_out(myGoalLink.x, myGoalLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);

    sprintf(imgFname, "Time = %.2f sec", (double)T * dt);
    putText(start_canvas, imgFname, cvPoint(45, 345), 1, 1, CV_RGB(0, 0, 0), 1, 8, false);

		sprintf(imgFname, "outfiles/animation/link_path_%d_%d_%d_screenshot_%d.png", 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, T);
		// imwrite(imgFname, start_canvas);
		imshow("Feed in animation", start_canvas);
    cvWaitKey(1);
  }

  myGoalLink.x += (1.0 - alt_Lks.back().expansion) * L * cos(robot.front().theta);
  myGoalLink.y += (1.0 - alt_Lks.back().expansion) * L * sin(robot.front().theta);

  // drill the second hole 2 ==================================================
  for(; robot.front().expansion < 1; T++){
    if ((1 - robot.front().expansion) > drillFeedSpeed * dt){
      //update position of LINKs of robot
      robot.front().expansion += drillFeedSpeed * dt / L;
      //update the head of robot
      HEAD.x += drillFeedSpeed * dt * cos(robot.front().theta);
      HEAD.y += drillFeedSpeed * dt * sin(robot.front().theta);
    }else{
      robot.front().expansion = 1.0;
      HEAD.x = robot.front().getHeadX();
      HEAD.y = robot.front().getHeadY();
    }
    cout << "Distance remaining: " << 1 - robot.front().expansion << endl;
    // plot the drilled hole
    line(realtime_display, cv_plot_coord_out(robot.front().x, robot.front().y),
        cv_plot_coord_out((robot.front().x + (1.8 * A + robot.front().expansion * L) * cos(robot.front().theta)), (robot.front().y + (1.8 * A + robot.front().expansion * L) * sin(robot.front().theta))), CV_RGB(255, 255, 255), 2 * PLOT_SCALE);

    start_canvas = realtime_display.clone();

    // plot robot links
    for (int i = 0; i < robot.size(); i++){
      robot[i].plotLINK(start_canvas);
    }

    circle(start_canvas, cv_plot_coord_out(myStartLink.x, myStartLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);
    circle(start_canvas, cv_plot_coord_out(myGoalLink.x, myGoalLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);

    //draw the broken line to the wall as the robot approaches
    swi = true;
    for (int i = 0; withinFig((int)round(robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE,
                                          (int)round(robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE)
                                          && realtime_display.at<Vec3b>((int)round(robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE,
                                                                                  (int)round(robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE)[0] > OBSTHRESHOLD; i++){
      if (swi){
        line(start_canvas, cv_plot_coord_out(robot.front().getHeadX() + i * cos(robot.front().theta), robot.front().getHeadY() + i * sin(robot.front().theta)),
            cv_plot_coord_out(robot.front().getHeadX() + (i + 1) * cos(robot.front().theta), robot.front().getHeadY() + (i + 1) * sin(robot.front().theta)), CV_RGB(100, 100, 100), LINE_THICKNESS, CV_AA);
      }
      swi = !swi;
    }

    sprintf(imgFname, "Time = %.2f sec", (double)T * dt);
    putText(start_canvas, imgFname, cvPoint(45, 345), 1, 1, CV_RGB(0, 0, 0), 1, 8, false);

    sprintf(imgFname, "outfiles/animation/link_path_%d_%d_%d_screenshot_%d.png", 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, T);
    // imwrite(imgFname, start_canvas);
    imshow("Feed in animation", start_canvas);
    cvWaitKey(1);
  }

  cout << "All animation finished!" << endl;
	cvWaitKey();

	return 0;
}
