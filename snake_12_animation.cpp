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
  return( cvPoint( PLOT_SCALE * x, PLOT_SCALE * y ) );
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
    /*cout << "gAccT = parentT + ((2 * A + L * expansion) * cos(theta + PI) * parentY - (2 * A + L * expansion) * sin(theta + PI) * parentX + (2 * A + L * expansion) / 2 * cos(theta + PI) * g[1] * mass - (2 * A + L * expansion) / 2 * sin(theta + PI) * g[0] * mass) * P2M_SCALE;" << endl;
    cout << gAccT << " = " << parentT << " + ((2 * " << A << " + " << L << " * " << expansion << ") * cos(" << theta << " + " << PI << ") * " << parentY << " - (2 * " << A << " + " << L << " * " << expansion << ") * sin(" << theta << " + " << PI << ") * " << parentX << " + (2 * " << A << " + " << L << " * " << expansion << ") / 2 * cos(" << theta << " + " << PI << ") * " << g[1] << " * " << mass << " - (2 * " << A << " + " << L << " * " << expansion << ") / 2 * sin(" << theta << " + " << PI << ") * " << g[0] << " * " << mass << ") * " << P2M_SCALE << endl;
    cout << gAccT << " = " << parentT << " + (" << (2 * A + L * expansion) * cos(theta + PI) << " * " << parentY << " - " << (2 * A + L * expansion) * cos(theta + PI) << " * " << parentX << " + " << (2 * A + L * expansion) / 2 * cos(theta + PI) << " * "<< g[1] << " * " << mass << " - " << (2 * A + L * expansion) / 2 * sin(theta + PI) << " * " << g[0] << " * " << mass << ") * " << P2M_SCALE << endl;*/
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


/*
void VSP_generator(string expt_f_name, string expt_name, string map_name, POINT s, POINT g){
  vector <POINT> landmarks;
  // set the landmarks
  if (map_name == "wall.png"){
    landmarks.push_back(s);
    landmarks.push_back(POINT(38, 143));
    landmarks.push_back(POINT(38, 143));
    landmarks.push_back(POINT(90, 143));
    landmarks.push_back(POINT(90, 36));
    landmarks.push_back(POINT(144, 36));
  }else if (map_name == "wall3.png"){
    landmarks.push_back(s);
  }else if (map_name == "wall_simu_1.png"){
    landmarks.push_back(s);
    landmarks.push_back(POINT(147, 167));
    landmarks.push_back(POINT(147, 267));
  }else if (map_name == "wall_simu_2.png"){
    landmarks.push_back(s);
    landmarks.push_back(POINT(92, 111));
  }else if (map_name == "wall_simu_3.png" || map_name == "wall_simu_3x.png"){
    landmarks.push_back(s);
    landmarks.push_back(POINT(161, 88));
    landmarks.push_back(POINT(161, 40));
  }

  // fill the points
  for(int i = 0; i < (landmarks.size() - 1); i++){
    VSP.push_back(landmarks[i]);

    double angle = atan2(landmarks[i + 1].y - landmarks[i].y, landmarks[i + 1].x - landmarks[i].x);
    double length = sqrt(pow(landmarks[i + 1].y - landmarks[i].y, 2) + pow(landmarks[i + 1].x - landmarks[i].x, 2));

    for (double j = 2; j < length; j += 2){
      VSP.push_back(POINT(landmarks[i].x + j * cos(angle), landmarks[i].y + j * sin(angle)));
    }
  }

  // last search section
  searchPOINT shortest_search(expt_f_name, expt_name, landmarks.back(), g);
  shortest_search.search();

  printf("shortest path search: start point: (%g, %g), goal point (%g, %g)\n", landmarks.back().x, landmarks.back().y, g.x, g.y);

  if (shortest_search.shortestGoals.empty()){
    cout << "No valid goal point reached!" << endl;
    return;
  }

  vector<POINT*> reconstructed_path = shortest_search.reconstructPointerPath(shortest_search.shortestGoals.back());

  if (reconstructed_path.empty()){
    cout << "No valid VSP constructed!" << endl;
    return;
  }
  // put the searched path into VSP
  for (int a = reconstructed_path.size() - 1; a >= 0; --a) {
    VSP.push_back(*reconstructed_path[a]);
  }
  if (VSP.back() != shortest_search.goalNode) VSP.push_back(shortest_search.goalNode);

  // draw the VSP and calculate the cost
  POINT thisPt, lastPt = VSP.front();
  double heuristicCost = 0.0;
  for (int a = 1; a < VSP.size(); a++){
    thisPt = VSP[a];
    if (round(PLOT_SCALE * (lastPt.x - shortest_search.MIN_X)) == 226 && round(PLOT_SCALE * (lastPt.y - shortest_search.MIN_Y)) == 379){
      cout << "singularity found at (226, 379) scaled" << endl;
    }
    shortest_search.cvPlotPoint(shortest_search.cv_plot_coord(lastPt.x, lastPt.y), CV_RGB(0, 165, 81), VERTEX_SIZE * 2);
    heuristicCost += sqrt(pow(thisPt.x - lastPt.x, 2) + pow(thisPt.y - lastPt.y, 2));
    lastPt = thisPt;
  }
  shortest_search.cvPlotPoint(shortest_search.cv_plot_coord(VSP.front().x, VSP.front().y), CV_RGB(255, 0, 0), VERTEX_SIZE * 3);
  shortest_search.cvPlotPoint(shortest_search.cv_plot_coord(VSP.back().x, VSP.back().y), CV_RGB(255, 0, 0), VERTEX_SIZE * 3);

  cout << "VSP size :" << VSP.size() << ", heuristic path cost: " << heuristicCost << endl;
  char fileName[1024];
  time_t now = time(0);
	tm* localtm = localtime(&now);
  sprintf(fileName, "outfiles/%d_%d_%d_%d_%d_shortest_search.png", 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, localtm->tm_hour, localtm->tm_min);

  imwrite(fileName, shortest_search.image_to_display);
  imshow("Display window", shortest_search.image_to_display);
  cvWaitKey();
}
*/
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
  if (part == 1){ // first hole, from entrance to hole
    path.push_back(LINK(125.005, 281.548, 0, 0));
    path.push_back(LINK(141.005, 281.548, 0, 0));
    path.push_back(LINK(157.005, 281.548, -0.785398, 0));
    path.push_back(LINK(168.319, 270.234, -1.5708, 0));
    path.push_back(LINK(168.319, 254.234, -2.35619, 0.5));
    path.push_back(LINK(154.088, 240.003, -2.74889, 1));
    path.push_back(LINK(131.684, 230.723, -1.9635, 0));
    path.push_back(LINK(125.561, 215.941, -1.5708, 0));
    path.push_back(LINK(125.561, 199.941, -0.785398, 0));
    path.push_back(LINK(136.875, 188.627, 0, 0.5));
    path.push_back(LINK(157, 188.627, -0.785398, 0));
    path.push_back(LINK(168.314, 177.314, -1.5708, 0));
    path.push_back(LINK(168.314, 161.314, -2.35619, 0));
    path.push_back(LINK(157, 150, -3.14159, 0));
    path.push_back(LINK(141, 150, -3.14159, 0));
  }else if (part == 2){ //  tapping
    path.push_back(LINK(125.385, 150, -3.14159, 0)); //150.334
    path.push_back(LINK(109.385, 150, -2.74889, 0.5));
    path.push_back(LINK(90.7918, 142.633, -2.35619, 0.5));
    path.push_back(LINK(76.5613, 128.402, -1.5708, 0));
    path.push_back(LINK(76.5613, 112.402, -0.785398, 0));
    path.push_back(LINK(87.875, 101.088, 0, 0.5));
    path.push_back(LINK(108, 101.088, -0.785398, 0));
    path.push_back(LINK(119.314, 89.7748, -1.5708, 0));
    path.push_back(LINK(119.314, 73.7748, -2.35619, 0.5));
    path.push_back(LINK(105.083, 59.5442, -3.14159, 0));
    path.push_back(LINK(89.0832, 59.5442, -2.35619, 0));
    path.push_back(LINK(77.7695, 48.2305, -1.5708, 0));
    path.push_back(LINK(77.7695, 32.2305, -0.785398, 0.5));
  }else if (part == 3){ // drilling
    path.push_back(LINK(125.794, 150, -3.14159, 0.6));//path.push_back(LINK(125.794, 149.581, -3.14159, 0.6));
    path.push_back(LINK(104.794, 150, -3.14159, 0));// 149.581
    path.push_back(LINK(88.7939, 150, -2.35619, 0));// 149.581
    path.push_back(LINK(77.4802, 138.267, -1.5708, 0));
    path.push_back(LINK(77.4802, 122.267, -0.785398, 0));
    path.push_back(LINK(88.7939, 110.954, 0, 0));
    path.push_back(LINK(104.794, 110.954, -0.785398, 0.5));
    path.push_back(LINK(119.024, 96.7232, -1.5708, 0));
    path.push_back(LINK(119.024, 80.7232, -2.35619, 0));
    path.push_back(LINK(107.711, 69.4095, -3.14159, 0));
    path.push_back(LINK(91.7107, 69.4095, -2.35619, 0.5));
    path.push_back(LINK(77.4802, 55.179, -1.5708, 0));
    path.push_back(LINK(77.4802, 39.179, -1.1781, 0));
    path.push_back(LINK(83.6031, 26.3969, -0.785398, 0));//24.3969
    for(auto &i : path){
      i.y += 1.0;
    }
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
	string expt_f_name = program_folderName + "exptfiles/experiments_11_animation.json", expt_name = "path_plan_1";

	if (argc > 1) {
		expt_f_name = argv[1];
		expt_name = argv[2];
	}

  // Read from file
  ifstream my_fstream (expt_f_name);
  RSJresource expt_container = RSJresource (my_fstream)[expt_name];
  POINT myStartLink(130, 281.54);
  POINT myGoalLink(125, 150);
  // Read map from file
  string expt_folderName = expt_f_name.substr(0, expt_f_name.find_last_of("/\\")+1);

  string map_image_fName = expt_folderName + expt_container["map_name"].as<std::string>();
  // cvParseMap2d my_map = cvParseMap2d (map_image_fName, false); // second parameter computes representative points
  // Mat spare_canvas = my_map.getCvMat(COLOR_MAP);
  Mat spare_canvas = imread(map_image_fName, CV_LOAD_IMAGE_COLOR);
  //Mat spare_canvas;
  resize(spare_canvas, spare_canvas, Size(), PLOT_SCALE, PLOT_SCALE, INTER_NEAREST);

  MAX_X = spare_canvas.cols;
  MAX_Y = spare_canvas.rows;

	vector<vector<LINK> > allLks;
  allLks.push_back(getConfig(1));
  allLks.push_back(getConfig(2));
  allLks[1].insert(allLks[1].begin(), allLks[0].begin(), allLks[0].end());
  allLks.push_back(getConfig(3));
  allLks[2].insert(allLks[2].begin(), allLks[0].begin(), allLks[0].end());

  // ===========================================================================
  //hereafter shows the final configuration
	Mat start_canvas = spare_canvas.clone();

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
	sprintf(imgFname, "outfiles/%s_%d_%d_%d_%d_%d_begin_%s.png", argv[0], 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, localtm->tm_hour, localtm->tm_min, _OS_FLAG == 1? "Linux" : "Mac");
	//imwrite(regex_replace(regex_replace(imgFname, regex("\n"), ""), regex(":"), "_"), start_canvas);
  cout << "final configuration shown" << endl;
	imshow("Feed in animation", start_canvas);
	cvWaitKey();
	start_canvas.release();

  //animation starts hereafter
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
		start_canvas = spare_canvas.clone();

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
		if(robot.empty() || robot.back().getPoint() == alt_Lks[1].getPoint()){
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
    for (int i = 0; spare_canvas.at<Vec3b>((int)round((robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE),
                                            (int)round((robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE))[0] > OBSTHRESHOLD; i++){
      if (swi){
        line(start_canvas, cv_plot_coord_out(robot.front().getHeadX() + i * cos(robot.front().theta), robot.front().getHeadY() + i * sin(robot.front().theta)),
            cv_plot_coord_out(robot.front().getHeadX() + (i + 1) * cos(robot.front().theta), robot.front().getHeadY() + (i + 1) * sin(robot.front().theta)), CV_RGB(100, 100, 100), LINE_THICKNESS, CV_AA);
      }
      swi = !swi;
    }

    circle(start_canvas, cv_plot_coord_out(myStartLink.x, myStartLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);
    circle(start_canvas, cv_plot_coord_out(myGoalLink.x, myGoalLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);

    sprintf(imgFname, "Time = %.2f sec", (double)T * dt);
    putText(start_canvas, imgFname, cvPoint(25, 830), 1, 1, CV_RGB(0, 0, 0), 1, 8, false);

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

  myGoalLink.x = robot.front().x + (2 * A + L) * cos(robot.front().theta);
  myGoalLink.y = robot.front().y + (2 * A + L) * sin(robot.front().theta);

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
    line(spare_canvas, cv_plot_coord_out(robot.front().x, robot.front().y),
				cv_plot_coord_out((robot.front().x + (1.8 * A + robot.front().expansion * L) * cos(robot.front().theta)), (robot.front().y + (1.8 * A + robot.front().expansion * L) * sin(robot.front().theta))), CV_RGB(255, 255, 255), 2 * PLOT_SCALE);

    start_canvas = spare_canvas.clone();

    circle(start_canvas, cv_plot_coord_out(myStartLink.x, myStartLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);
    circle(start_canvas, cv_plot_coord_out(myGoalLink.x, myGoalLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);

    // plot robot links
    for (int i = 0; i < robot.size(); i++){
      robot[i].plotLINK(start_canvas);
    }

    //draw the broken line to the wall as the robot approaches
    swi = true;
    for (int i = 0; spare_canvas.at<Vec3b>((int)round((robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE),
                                            (int)round((robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE))[0] > OBSTHRESHOLD; i++){
      if (swi){
        line(start_canvas, cv_plot_coord_out(robot.front().getHeadX() + i * cos(robot.front().theta), robot.front().getHeadY() + i * sin(robot.front().theta)),
            cv_plot_coord_out(robot.front().getHeadX() + (i + 1) * cos(robot.front().theta), robot.front().getHeadY() + (i + 1) * sin(robot.front().theta)), CV_RGB(100, 100, 100), LINE_THICKNESS, CV_AA);
      }
      swi = !swi;
    }

    sprintf(imgFname, "Time = %.2f sec", (double)T * dt);
    putText(start_canvas, imgFname, cvPoint(25, 830), 1, 1, CV_RGB(0, 0, 0), 1, 8, false);

		sprintf(imgFname, "outfiles/animation/link_path_%d_%d_%d_screenshot_%d.png", 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, T);
		// imwrite(imgFname, start_canvas);
		imshow("Feed in animation", start_canvas);
		cvWaitKey(1);
  }

  myGoalLink = allLks[1].back().getHead();//POINT(92, 17);
// approaching 2 ===============================================================

  // set the alternative links for guidance
  alt_Lks = allLks[1];
  // insert a external link as new fed-in
  alt_Lks.insert(alt_Lks.begin(), LINK(alt_Lks.front().x - 2 * A * cos(alt_Lks.front().theta), alt_Lks.front().y - 2 * A * sin(alt_Lks.front().theta), alt_Lks.front().theta, 0));

  for(; robot.size() < (24 - shift); T++){ //24
    //reset canvas
    start_canvas = spare_canvas.clone();

    //shortcuts
    for(int b = 0; b < (allLks[1].size() - 1); b++){
      line(start_canvas, cv_plot_coord_out(allLks[1][b].x, allLks[1][b].y),
          cv_plot_coord_out(allLks[1][b + 1].getHeadX(), allLks[1][b + 1].getHeadY()), CV_RGB(255, 153, 51), LINE_THICKNESS, CV_AA);
    }

    //draw auxilary configuration in light green color
    for(auto b : alt_Lks){
      line(start_canvas, cv_plot_coord_out(b.x, b.y),
          cv_plot_coord_out((b.x + (2 * A + b.expansion * L) * cos(b.theta)), (b.y + (2 * A + b.expansion * L) * sin(b.theta))), CV_RGB(153, 255, 204), LINE_THICKNESS * PLOT_SCALE, CV_AA);
    }

    //draw final configuration in dark color
    for(auto b : allLks[1]){
      line(start_canvas, cv_plot_coord_out(b.x, b.y),
          cv_plot_coord_out((b.x + (2 * A + b.expansion * L) * cos(b.theta)), (b.y + (2 * A + b.expansion * L) * sin(b.theta))), CV_RGB(153, 204, 255), LINE_THICKNESS * PLOT_SCALE, CV_AA);
    }

    //move the robot

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
        robot[it].expansion = (sqrt(pow(robot[it - 1].x - robot[it].x, 2) + pow(robot[it - 1].y - robot[it].y, 2)) - 2 * A) / L;
      }else{
        robot[it].theta = atan2(HEAD.y - robot[it].y, HEAD.x - robot[it].x);
        robot[it].expansion = (sqrt(pow(HEAD.x - robot[it].x, 2) + pow(HEAD.y - robot[it].y, 2)) - 2 * A) / L;
      }
    }

    if (dist(HEAD, alt_Lks[cur_po].getHead()) < INFINITESIMAL_DOUBLE){
      printf("cur_po++: %d\n", ++cur_po);
    }

    //add new LINKs
    if(robot.back().getPoint() == alt_Lks[1].getPoint()){
      robot.push_back(alt_Lks.front());
      printf("new-insert LINK: coordinates (%g, %g), theta: %g PI\n", alt_Lks.front().x, alt_Lks.front().y, alt_Lks.front().theta / PI);
    }

    printf("Animation time: %g sec\n", (double)T * dt);

    // plot robot links
    for (int i = 0; i < robot.size(); i++){
      robot[i].plotLINK(start_canvas);
    }

    //draw the broken line to the wall as the robot approaches
    swi = true;
    for (int i = 0; spare_canvas.at<Vec3b>((int)round((robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE),
                                            (int)round((robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE))[0] > OBSTHRESHOLD; i++){
      if (swi){
        line(start_canvas, cv_plot_coord_out(robot.front().getHeadX() + i * cos(robot.front().theta), robot.front().getHeadY() + i * sin(robot.front().theta)),
            cv_plot_coord_out(robot.front().getHeadX() + (i + 1) * cos(robot.front().theta), robot.front().getHeadY() + (i + 1) * sin(robot.front().theta)), CV_RGB(100, 100, 100), LINE_THICKNESS, CV_AA);
      }
      swi = !swi;
    }

    circle(start_canvas, cv_plot_coord_out(myStartLink.x, myStartLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);
    circle(start_canvas, cv_plot_coord_out(myGoalLink.x, myGoalLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);

    sprintf(imgFname, "Time = %.2f sec", (double)T * dt);
    putText(start_canvas, imgFname, cvPoint(25, 830), 1, 1, CV_RGB(0, 0, 0), 1, 8, false);

    sprintf(imgFname, "outfiles/animation/link_path_%d_%d_%d_screenshot_%d.png", 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, T);
    // imwrite(imgFname, start_canvas);
    imshow("Feed in animation", start_canvas);
    cvWaitKey(1);
  }

// shrink links from drilling the first hole ====================================
bool allShrunk = false;
for(; !allShrunk; T++){
  //reset canvas
  start_canvas = spare_canvas.clone();

  //shortcuts
  for(int b = 0; b < (allLks[1].size() - 1); b++){
    line(start_canvas, cv_plot_coord_out(allLks[1][b].x, allLks[1][b].y),
        cv_plot_coord_out(allLks[1][b + 1].getHeadX(), allLks[1][b + 1].getHeadY()), CV_RGB(255, 153, 51), LINE_THICKNESS, CV_AA);
  }

  //draw auxilary configuration in light green color
  for(auto b : alt_Lks){
    line(start_canvas, cv_plot_coord_out(b.x, b.y),
        cv_plot_coord_out((b.x + (2 * A + b.expansion * L) * cos(b.theta)), (b.y + (2 * A + b.expansion * L) * sin(b.theta))), CV_RGB(153, 255, 204), LINE_THICKNESS * PLOT_SCALE, CV_AA);
  }

  //draw final configuration in dark color
  for(auto b : allLks[1]){
    line(start_canvas, cv_plot_coord_out(b.x, b.y),
        cv_plot_coord_out((b.x + (2 * A + b.expansion * L) * cos(b.theta)), (b.y + (2 * A + b.expansion * L) * sin(b.theta))), CV_RGB(153, 204, 255), LINE_THICKNESS * PLOT_SCALE, CV_AA);
  }

  //move LINK 13
  if (robot[13 - shift].expansion > 0.0){
    robot[13 - shift].expansion -= 1.0 / steps_per_LINK;
    if (robot[13 - shift].expansion < 0.0) robot[13 - shift].expansion = 0.0;
  }
  LINK ne = nextPos(robot[13 - shift], robot[12 - shift]);
  robot[13 - shift].x = ne.x;
  robot[13 - shift].y = ne.y;

  // link 14
  ne = nextPos(robot[14 - shift], robot[13 - shift]);
  robot[14 - shift].x = ne.x;
  robot[14 - shift].y = ne.y;

  // link 15
  ne = nextPos(robot[15 - shift], robot[14 - shift]);
  robot[15 - shift].x = ne.x;
  robot[15 - shift].y = ne.y;


  // link 18
  if (robot[18 - shift].expansion > 0.0){
    robot[18 - shift].expansion -= 1.0 / steps_per_LINK;
    if (robot[18 - shift].expansion < 0.0) robot[18 - shift].expansion = 0.0;
  }

  // link 17
  if (robot[17 - shift].expansion > 0.0){
    robot[17 - shift].expansion -= 1.0 / steps_per_LINK;
    if (robot[17 - shift].expansion < 0.0) robot[17 - shift].expansion = 0.0;
  }

  // link 16, 17
  double d = dist(robot[15 - shift].getPoint(), robot[18 - shift].getHead());
  robot[17 - shift].x = robot[18 - shift].getHeadX();
  robot[17 - shift].y = robot[18 - shift].getHeadY();
  if (d <= (robot[16 - shift].getLength() + robot[17 - shift].getLength())){//} && fabs(robot[15 - shift].theta - robot[16 - shift].theta) <= (PI / 4)){
    POINT pm = triCorner(robot[15 - shift].getPoint(), robot[18 - shift].getHead(), robot[16 - shift].getLength(), robot[17 - shift].getLength(), d);
    robot[16 - shift].x = pm.x;
    robot[16 - shift].y = pm.y;
  }else{
    POINT pm = triCorner(robot[13 - shift].getHead(), robot[17 - shift].getPoint(), dist(robot[13 - shift].getHead(),robot[15 - shift].getPoint()), robot[16 - shift].getLength() + robot[17 - shift].getLength(), dist(robot[13 - shift].getHead(),robot[18 - shift].getHead()));
    robot[15 - shift].x = pm.x;
    robot[15 - shift].y = pm.y;
    POINT pm1 = triCorner(robot[13 - shift].getHead(), robot[15 - shift].getPoint(), dist(robot[13 - shift].getHead(), robot[14 - shift].getPoint()), robot[15 - shift].getLength(), dist(robot[13 - shift].getHead(), robot[15 - shift].getPoint()));
    robot[14 - shift].x = pm1.x;
    robot[14 - shift].y = pm1.y;
    POINT pm2 = triCorner(robot[13 - shift].getHead(), robot[14 - shift].getPoint(), robot[13 - shift].getLength(), robot[14 - shift].getLength(), dist(robot[13 - shift].getHead(),robot[14 - shift].getPoint()));
    robot[13 - shift].x = pm2.x;
    robot[13 - shift].y = pm2.y;
    robot[13 - shift].theta = atan2(robot[12 - shift].y - robot[13 - shift].y, robot[12 - shift].x - robot[13 - shift].x);
    robot[14 - shift].theta = atan2(robot[13 - shift].y - robot[14 - shift].y, robot[13 - shift].x - robot[14 - shift].x);
    robot[15 - shift].theta = atan2(robot[14 - shift].y - robot[15 - shift].y, robot[14 - shift].x - robot[15 - shift].x);

    robot[16 - shift].x = robot[17 - shift].x + (robot[15 - shift].x - robot[17 - shift].x) * robot[17 - shift].getLength() / (robot[16 - shift].getLength() + robot[17 - shift].getLength());
    robot[16 - shift].y = robot[17 - shift].y + (robot[15 - shift].y - robot[17 - shift].y) * robot[17 - shift].getLength() / (robot[16 - shift].getLength() + robot[17 - shift].getLength());
  }

  robot[16 - shift].theta = atan2(robot[15 - shift].y - robot[16 - shift].y, robot[15 - shift].x - robot[16 - shift].x);
  robot[17 - shift].theta = atan2(robot[16 - shift].y - robot[17 - shift].y, robot[16 - shift].x - robot[17 - shift].x);

  if (robot[13 - shift].expansion == 0.0 && robot[17 - shift].expansion == 0.0 && robot[18 - shift].expansion == 0.0) allShrunk = true;

  printf("Animation time: %g sec\n", (double)T * dt);


  // plot robot links
  for (int i = 0; i < robot.size(); i++){
    robot[i].plotLINK(start_canvas);
  }

  //draw the broken line to the wall as the robot approaches
  swi = true;
  for (int i = 0; spare_canvas.at<Vec3b>((int)round((robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE),
                                            (int)round((robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE))[0] > OBSTHRESHOLD; i++){
    if (swi){
      line(start_canvas, cv_plot_coord_out(robot.front().getHeadX() + i * cos(robot.front().theta), robot.front().getHeadY() + i * sin(robot.front().theta)),
          cv_plot_coord_out(robot.front().getHeadX() + (i + 1) * cos(robot.front().theta), robot.front().getHeadY() + (i + 1) * sin(robot.front().theta)), CV_RGB(100, 100, 100), LINE_THICKNESS, CV_AA);
    }
    swi = !swi;
  }

  circle(start_canvas, cv_plot_coord_out(myStartLink.x, myStartLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);
  circle(start_canvas, cv_plot_coord_out(myGoalLink.x, myGoalLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);

  sprintf(imgFname, "Time = %.2f sec", (double)T * dt);
  putText(start_canvas, imgFname, cvPoint(25, 830), 1, 1, CV_RGB(0, 0, 0), 1, 8, false);

  sprintf(imgFname, "outfiles/animation/link_path_%d_%d_%d_screenshot_%d.png", 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, T);
  // imwrite(imgFname, start_canvas);
  imshow("Feed in animation", start_canvas);
  cvWaitKey(1);
}

// approaching to the second hole =============================================
  for(; robot.front().getHead() != alt_Lks.back().getHead(); T++){
    //reset canvas
    start_canvas = spare_canvas.clone();

    //shortcuts
    for(int b = 0; b < (allLks[1].size() - 1); b++){
      line(start_canvas, cv_plot_coord_out(allLks[1][b].x, allLks[1][b].y),
          cv_plot_coord_out(allLks[1][b + 1].getHeadX(), allLks[1][b + 1].getHeadY()), CV_RGB(255, 153, 51), LINE_THICKNESS, CV_AA);
    }

    //draw auxilary configuration in light green color
    for(auto b : alt_Lks){
      line(start_canvas, cv_plot_coord_out(b.x, b.y),
          cv_plot_coord_out((b.x + (2 * A + b.expansion * L) * cos(b.theta)), (b.y + (2 * A + b.expansion * L) * sin(b.theta))), CV_RGB(153, 255, 204), LINE_THICKNESS * PLOT_SCALE, CV_AA);
    }

    //draw final configuration in dark color
    for(auto b : allLks[1]){
      line(start_canvas, cv_plot_coord_out(b.x, b.y),
          cv_plot_coord_out((b.x + (2 * A + b.expansion * L) * cos(b.theta)), (b.y + (2 * A + b.expansion * L) * sin(b.theta))), CV_RGB(153, 204, 255), LINE_THICKNESS * PLOT_SCALE, CV_AA);
    }

    //move the robot

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
      if (robot[it].x <= 141 && robot[it].y <= 151){
        robot[it].x += (dists[it].x / steps_per_LINK);
        robot[it].y += (dists[it].y / steps_per_LINK);
      }else if (robot[it - 1].x <= 141 && robot[it - 1].y <= 151){
          robot[it].x += (dists[it].x / steps_per_LINK);
          robot[it].y += (dists[it].y / steps_per_LINK);
          printf("next one moved: it = %d\n", it);
      }else if (it == (robot.size() - 2)){
        robot[it].x = robot[it - 1].x - sqrt(pow(2 * A + L * robot[it].expansion, 2) - pow(robot[it - 1].y - robot[it].y, 2));
      }else{
        if (it != (robot.size() - 1)){
          LINK ne = nextPos(robot[it], robot[it - 1]);
          robot[it].x = ne.x;
          robot[it].y = ne.y;
        }else{
          robot[it].x = robot[it - 1].x - robot[it].getLength();
        }
      }

      if (it > 0){
        robot[it].theta = atan2(robot[it - 1].y - robot[it].y, robot[it - 1].x - robot[it].x);
        robot[it].expansion = (sqrt(pow(robot[it - 1].x - robot[it].x, 2) + pow(robot[it - 1].y - robot[it].y, 2)) - 2 * A) / L;
      }else{
        robot[it].theta = atan2(HEAD.y - robot[it].y, HEAD.x - robot[it].x);
        robot[it].expansion = (sqrt(pow(HEAD.x - robot[it].x, 2) + pow(HEAD.y - robot[it].y, 2)) - 2 * A) / L;
      }
    }

    double delta_t = robot.rbegin()[1].theta - robot.rbegin()[0].theta;
    int outit;
    for(outit = 1; fabs(delta_t) > (PI / 4); outit++){
      robot.rbegin()[outit].x = robot.rbegin()[outit - 1].getHeadX();
      robot.rbegin()[outit].y = robot.rbegin()[outit - 1].getHeadY();
      robot.rbegin()[outit].theta = robot.rbegin()[outit - 1].theta + (delta_t > 0? 1 : -1) * PI / 4; // may need to be revised
      delta_t = robot.rbegin()[outit + 1].theta - robot.rbegin()[outit].theta;
    }
    // fix the expansion of the middle link
    robot.rbegin()[outit].x = robot.rbegin()[outit - 1].getHeadX();
    robot.rbegin()[outit].y = robot.rbegin()[outit - 1].getHeadY();
    if (fabs(delta_t) != (PI / 4)){
      robot.rbegin()[outit].theta = atan2(robot.rbegin()[outit + 1].y - robot.rbegin()[outit].y, robot.rbegin()[outit + 1].x - robot.rbegin()[outit].x);
    }
    robot.rbegin()[outit].expansion = (dist(robot.rbegin()[outit].getPoint(), robot.rbegin()[outit + 1].getPoint()) - 2 * A) / L;

    if (dist(HEAD, alt_Lks[cur_po].getHead()) < INFINITESIMAL_DOUBLE){
      printf("cur_po++: %d\n", ++cur_po);
    }

    //add new LINKs
    if(robot.back().x >= alt_Lks[1].x){
      robot.push_back(alt_Lks.front());
      printf("new-insert LINK: coordinates (%g, %g), theta: %g PI\n", alt_Lks.front().x, alt_Lks.front().y, alt_Lks.front().theta / PI);
    }

    printf("Animation time: %g sec\n", (double)T * dt);

    if(robot.back().getPoint() == alt_Lks[1].getPoint()){
			robot.push_back(alt_Lks.front());
      printf("new-insert LINK: coordinates (%g, %g), theta: %g PI\n", alt_Lks.front().x, alt_Lks.front().y, alt_Lks.front().theta / PI);
		}


    // plot robot links
    for (int i = 0; i < robot.size(); i++){
      robot[i].plotLINK(start_canvas);
    }

    //draw the broken line to the wall as the robot approaches
    swi = true;
    for (int i = 0; spare_canvas.at<Vec3b>((int)round((robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE),
                                            (int)round((robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE))[0] > OBSTHRESHOLD; i++){
      if (swi){
        line(start_canvas, cv_plot_coord_out(robot.front().getHeadX() + i * cos(robot.front().theta), robot.front().getHeadY() + i * sin(robot.front().theta)),
            cv_plot_coord_out(robot.front().getHeadX() + (i + 1) * cos(robot.front().theta), robot.front().getHeadY() + (i + 1) * sin(robot.front().theta)), CV_RGB(100, 100, 100), LINE_THICKNESS, CV_AA);
      }
      swi = !swi;
    }

    circle(start_canvas, cv_plot_coord_out(myStartLink.x, myStartLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);
    circle(start_canvas, cv_plot_coord_out(myGoalLink.x, myGoalLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);

    sprintf(imgFname, "Time = %.2f sec", (double)T * dt);
    putText(start_canvas, imgFname, cvPoint(25, 830), 1, 1, CV_RGB(0, 0, 0), 1, 8, false);

    sprintf(imgFname, "outfiles/animation/link_path_%d_%d_%d_screenshot_%d.png", 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, T);
    // imwrite(imgFname, start_canvas);
    imshow("Feed in animation", start_canvas);
    cvWaitKey(1);
  }

  myGoalLink.x = robot.front().x + (2 * A + L) * cos(robot.front().theta);
  myGoalLink.y = robot.front().y + (2 * A + L) * sin(robot.front().theta);

  // drill the second hole 1 ==================================================
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
    line(spare_canvas, cv_plot_coord_out(robot.front().x, robot.front().y),
        cv_plot_coord_out((robot.front().x + (1.8 * A + robot.front().expansion * L) * cos(robot.front().theta)), (robot.front().y + (1.8 * A + robot.front().expansion * L) * sin(robot.front().theta))), CV_RGB(255, 255, 255), 2 * PLOT_SCALE);

    start_canvas = spare_canvas.clone();

    circle(start_canvas, cv_plot_coord_out(myStartLink.x, myStartLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);
    circle(start_canvas, cv_plot_coord_out(myGoalLink.x, myGoalLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);

    // plot robot links
    for (int i = 0; i < robot.size(); i++){
      robot[i].plotLINK(start_canvas);
    }

    //draw the broken line to the wall as the robot approaches
    swi = true;
    for (int i = 0; spare_canvas.at<Vec3b>((int)round((robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE),
                                            (int)round((robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE))[0] > OBSTHRESHOLD; i++){
      if (swi){
        line(start_canvas, cv_plot_coord_out(robot.front().getHeadX() + i * cos(robot.front().theta), robot.front().getHeadY() + i * sin(robot.front().theta)),
            cv_plot_coord_out(robot.front().getHeadX() + (i + 1) * cos(robot.front().theta), robot.front().getHeadY() + (i + 1) * sin(robot.front().theta)), CV_RGB(100, 100, 100), LINE_THICKNESS, CV_AA);
      }
      swi = !swi;
    }

    sprintf(imgFname, "Time = %.2f sec", (double)T * dt);
    putText(start_canvas, imgFname, cvPoint(25, 830), 1, 1, CV_RGB(0, 0, 0), 1, 8, false);

    sprintf(imgFname, "outfiles/animation/link_path_%d_%d_%d_screenshot_%d.png", 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, T);
    // imwrite(imgFname, start_canvas);
    imshow("Feed in animation", start_canvas);
    cvWaitKey(1);
  }

// transfrom ========================================

  dists.resize(robot.size());
  for(int it = 0; it < dists.size(); it++){
    dists[it].x = allLks[2].rbegin()[it].x - robot[it].x;
    dists[it].y = allLks[2].rbegin()[it].y - robot[it].y;
  }

  for(; robot.front().getPoint() != allLks[2].back().getPoint(); T++){
    // the rest of the robot approaches
    cout << "Approaching" << endl;
    //update position of LINKs of robot
    for(int it = 0; it < robot.size(); it++){
      if (robot[it].x <= 141 && robot[it].y <= 151){
        robot[it].x += (dists[it].x / steps_per_LINK);
        robot[it].y += (dists[it].y / steps_per_LINK);
      }else if (robot[it - 1].x <= 141 && robot[it - 1].y <= 151){
        robot[it].x += (dists[it].x / steps_per_LINK);
        robot[it].y += (dists[it].y / steps_per_LINK);
        printf("previous one: it - 1 = %d (%g, %g)\n", it - 1, robot[it - 1].x, robot[it - 1].y);
        printf("current one moved: it = %d\n", it);
        printf("next one: it + 1 = %d (%g, %g)\n", it + 1, robot[it + 1].x, robot[it + 1].y);
      }else if (it == (robot.size() - 2)){
        robot[it].x = robot[it - 1].x - sqrt(pow(2 * A + L * robot[it].expansion, 2) - pow(robot[it - 1].y - robot[it].y, 2));
      }else{
        if (it != (robot.size() - 1)){
          LINK ne = nextPos(robot[it], robot[it - 1]);
          robot[it].x = ne.x;
          robot[it].y = ne.y;
        }else{
          robot[it].x = robot[it - 1].x - robot[it].getLength();
        }
      }

      if (it > 0){
        robot[it].theta = atan2(robot[it - 1].y - robot[it].y, robot[it - 1].x - robot[it].x);
        robot[it].expansion = (sqrt(pow(robot[it - 1].x - robot[it].x, 2) + pow(robot[it - 1].y - robot[it].y, 2)) - 2 * A) / L;
      }else{
        robot[it].theta = atan2(HEAD.y - robot[it].y, HEAD.x - robot[it].x);
        robot[it].expansion = (sqrt(pow(HEAD.x - robot[it].x, 2) + pow(HEAD.y - robot[it].y, 2)) - 2 * A) / L;
      }
    }

    //double t = atan2(robot.rbegin()[2].y - robot.rbegin()[1].y, robot.rbegin()[2].x - robot.rbegin()[1].x) - atan2(robot.rbegin()[1].y - robot.rbegin()[0].y, robot.rbegin()[1].x - robot.rbegin()[0].x);
    double delta_t = robot.rbegin()[1].theta - robot.rbegin()[0].theta;
    int outit;
    for(outit = 1; fabs(delta_t) > (PI / 4); outit++){
      robot.rbegin()[outit].x = robot.rbegin()[outit - 1].getHeadX();
      robot.rbegin()[outit].y = robot.rbegin()[outit - 1].getHeadY();
      robot.rbegin()[outit].theta = robot.rbegin()[outit - 1].theta + (delta_t > 0? 1 : -1) * PI / 4; // may need to be revised
      delta_t = robot.rbegin()[outit + 1].theta - robot.rbegin()[outit].theta;
    }
    // fix the expansion of the middle link
    robot.rbegin()[outit].x = robot.rbegin()[outit - 1].getHeadX();
    robot.rbegin()[outit].y = robot.rbegin()[outit - 1].getHeadY();
    if (fabs(delta_t) != (PI / 4)){
      robot.rbegin()[outit].theta = atan2(robot.rbegin()[outit + 1].y - robot.rbegin()[outit].y, robot.rbegin()[outit + 1].x - robot.rbegin()[outit].x);
    }
    robot.rbegin()[outit].expansion = (dist(robot.rbegin()[outit].getPoint(), robot.rbegin()[outit + 1].getPoint()) - 2 * A) / L;
    if (robot.rbegin()[outit].expansion > 1.0){
      robot.rbegin()[outit].expansion = 1.0;
      robot.rbegin()[outit + 1].x = robot.rbegin()[outit].getHeadX();
      robot.rbegin()[outit + 1].y = robot.rbegin()[outit].getHeadY();
      robot.rbegin()[outit + 1].theta = atan2(robot.rbegin()[outit + 2].y - robot.rbegin()[outit + 1].y, robot.rbegin()[outit + 2].x - robot.rbegin()[outit + 1].x);
      robot.rbegin()[outit + 1].expansion = (dist(robot.rbegin()[outit + 2].getPoint(), robot.rbegin()[outit + 1].getPoint()) - 2 * A) / L;
    }

    //reset canvas
    start_canvas = spare_canvas.clone();

    //draw final configuration in light blue color
    for(auto b : allLks[2]){
      line(start_canvas, cv_plot_coord_out(b.x, b.y),
          cv_plot_coord_out((b.x + (2 * A + b.expansion * L) * cos(b.theta)), (b.y + (2 * A + b.expansion * L) * sin(b.theta))), CV_RGB(153, 204, 255), LINE_THICKNESS * PLOT_SCALE, CV_AA);
    }

    printf("Animation time: %g sec\n", (double)T * dt);

    if(robot.back().x >= alt_Lks[1].x){
			robot.push_back(alt_Lks.front());
      printf("new-insert LINK: coordinates (%g, %g), theta: %g PI\n", alt_Lks.front().x, alt_Lks.front().y, alt_Lks.front().theta / PI);
		}

    /// plot robot links
    for (int i = 0; i < robot.size(); i++){
      robot[i].plotLINK(start_canvas);
    }

    //draw the broken line to the wall as the robot approaches
    swi = true;
    for (int i = 0; spare_canvas.at<Vec3b>((int)round((robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE),
                                            (int)round((robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE))[0] > OBSTHRESHOLD; i++){
      if (swi){
        line(start_canvas, cv_plot_coord_out(robot.front().getHeadX() + i * cos(robot.front().theta), robot.front().getHeadY() + i * sin(robot.front().theta)),
            cv_plot_coord_out(robot.front().getHeadX() + (i + 1) * cos(robot.front().theta), robot.front().getHeadY() + (i + 1) * sin(robot.front().theta)), CV_RGB(100, 100, 100), LINE_THICKNESS, CV_AA);
      }
      swi = !swi;
    }

    circle(start_canvas, cv_plot_coord_out(myStartLink.x, myStartLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);
    circle(start_canvas, cv_plot_coord_out(myGoalLink.x, myGoalLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);

    sprintf(imgFname, "Time = %.2f sec", (double)T * dt);
    putText(start_canvas, imgFname, cvPoint(25, 830), 1, 1, CV_RGB(0, 0, 0), 1, 8, false);

		sprintf(imgFname, "outfiles/animation/link_path_%d_%d_%d_screenshot_%d.png", 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, T);
		// imwrite(imgFname, start_canvas);
		imshow("Feed in animation", start_canvas);
    cvWaitKey(1);
  }

  myGoalLink.x = robot.front().x + (2 * A + L) * cos(robot.front().theta);
  myGoalLink.y = robot.front().y + (2 * A + L) * sin(robot.front().theta);

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
    line(spare_canvas, cv_plot_coord_out(robot.front().x, robot.front().y),
        cv_plot_coord_out((robot.front().x + (1.8 * A + robot.front().expansion * L) * cos(robot.front().theta)), (robot.front().y + (1.8 * A + robot.front().expansion * L) * sin(robot.front().theta))), CV_RGB(255, 255, 255), 2 * PLOT_SCALE);

    start_canvas = spare_canvas.clone();

    circle(start_canvas, cv_plot_coord_out(myStartLink.x, myStartLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);
    circle(start_canvas, cv_plot_coord_out(myGoalLink.x, myGoalLink.y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, 8);

    // plot robot links
    for (int i = 0; i < robot.size(); i++){
      robot[i].plotLINK(start_canvas);
    }

    //draw the broken line to the wall as the robot approaches
    swi = true;
    for (int i = 0; withinFig((int)round((robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE),
                              (int)round((robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE))
                    && spare_canvas.at<Vec3b>((int)round((robot.front().getHeadY() + i * sin(robot.front().theta)) * PLOT_SCALE),
                                            (int)round((robot.front().getHeadX() + i * cos(robot.front().theta)) * PLOT_SCALE))[0] > OBSTHRESHOLD; i++){
      if (swi){
        line(start_canvas, cv_plot_coord_out(robot.front().getHeadX() + i * cos(robot.front().theta), robot.front().getHeadY() + i * sin(robot.front().theta)),
            cv_plot_coord_out(robot.front().getHeadX() + (i + 1) * cos(robot.front().theta), robot.front().getHeadY() + (i + 1) * sin(robot.front().theta)), CV_RGB(100, 100, 100), LINE_THICKNESS, CV_AA);
      }
      swi = !swi;
    }

    sprintf(imgFname, "Time = %.2f sec", (double)T * dt);
    putText(start_canvas, imgFname, cvPoint(25, 830), 1, 1, CV_RGB(0, 0, 0), 1, 8, false);

    sprintf(imgFname, "outfiles/animation/link_path_%d_%d_%d_screenshot_%d.png", 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, T);
    // imwrite(imgFname, start_canvas);
    imshow("Feed in animation", start_canvas);
    cvWaitKey(1);
  }

  cout << "All animation finished!" << endl;
	cvWaitKey();

	return 0;
}
