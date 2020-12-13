#ifndef CONFIGSEARCH
#define CONFIGSEARCH

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include <dosl/dosl>
#include "local-include/RSJparser.tcc"
#include "parameters.hpp"
#include "LINK.hpp"
#include "POINT.hpp"
#include <qpOASES.hpp> // quadratic programming

using namespace std;
using namespace cv;
USING_NAMESPACE_QPOASES

extern vector<POINT> sk_cost;
extern vector<POINT> sk_heu;
extern vector<LINK> allLks;
extern Point ** SkeletonMat;

class SEG {
public:
	POINT pt1;
	POINT pt2;
	SEG() {};
	SEG(POINT p1, POINT p2)
	{
		this->pt1 = p1;
		this->pt2 = p2;
	};

	bool operator == (const SEG & n) const {
		if ((pt1 == n.pt1 && pt2 == n.pt2) || (pt1 == n.pt2 && pt2 == n.pt1)) return (true);
		return (false);
	}

};

class searchLINK : public AStar::Algorithm<searchLINK, LINK, double>{
//class searchLINK : public AStarProblem<LINK>{
public:
	// File names and JSON objects
	string map_image_fName, expt_fName, expt_folderName, expt_Name, out_folderName;
	char out_Name[1024];
	RSJresource expt_container;
  Mat original_map;
  Mat color_map;
	Mat grey_map;

	// Image display variables / parameters
	Mat realtime_display;
	//double PLOT_SCALE;
	//double VERTEX_SIZE, LINE_THICKNESS;
	clock_t start_time;
	double duration;
	int mins, flagmin, SAVE_FLAG;
  unsigned long int n_successors; // number of successor expanded

	// variables decsribing problem
	int MAX_X, MIN_X, MAX_Y, MIN_Y;
  LINK startLink, goalLink, drillLink;
  int searchMode; // 0 = first search, 1 = 2nd search, 2 = 1st transitional search

	// solution classes
	vector<LINK> paths;
  vector<LINK> over_torque_links;
  //Point ** SkeletonMat;

	void cvPlotPoint (Mat & canvas, CvPoint pt, CvScalar colr, int size) {
		for (int i = (pt.x - (int)((size * 2 - 1) / 2)); i <= (pt.x + (int)ceil((size * 2 - 1) / 2)); i++){
			for (int j = (pt.y - (int)((size * 2 - 1) / 2)); j <= (pt.y + (int)ceil((size * 2 - 1) / 2)); j++){
				if (i < 0 || i >= canvas.cols || j < 0 || j >= canvas.rows) continue;
				canvas.at<Vec3b>(j,i) = Vec3b((uchar)colr.val[0], (uchar)colr.val[1], (uchar)colr.val[2]);
			}
		}
	}

  Point PresetClosestPt(Point p, Mat map){ // get the closest point of sk_cost/sk_deu to the end of this link
    // get the closest point on skeleton of cost
    int closestPtX = -1;
    double dis = 10000;
    for (int a = 0; a < sk_cost.size(); a++){
      bool isBlocked = false;
      double dx = sk_cost[a].x - p.x;
      double dy = sk_cost[a].y - p.y;
      double ang = atan2(dy, dx);
      double len = sqrt(dx * dx + dy * dy);

      // check if any wall between the head of link and the point
      for(double i = 1; i <= len; i++){
        if (map.at<Vec3b>((int)round((double)p.y + i * sin(ang)),
        (int)round((double)p.x + i * cos(ang))).val[0] <= OBSTHRESHOLD){
          isBlocked = true;
          break;
        }
      }
      // if no wall in between
      if (!isBlocked){
        if (dis > len){
          dis = len;
          closestPtX = a;
        }
      }
    }

    // if there's no closest point, it returns -1

    // get the closest point on skeleton of cost
    int closestPtY = -1;
    dis = 10000;
    for (int a = 0; a < sk_heu.size(); a++){
      bool isBlocked = false;
      double dx = sk_heu[a].x - p.x;
      double dy = sk_heu[a].y - p.y;
      double ang = atan2(dy, dx);
      double len = sqrt(dx * dx + dy * dy);

      // check if any wall between the head of link and the point
      for(double i = 1; i <= len; i++){
        if (map.at<Vec3b>((int)round(p.y + i * sin(ang)),
        (int)round(p.x + i * cos(ang))).val[0] <= OBSTHRESHOLD){
          isBlocked = true;
          break;
        }
      }
      // if no wall in between
      if (!isBlocked){
        if (dis > len){
          dis = len;
          closestPtY = a;
        }
      }
    }

    // if there's no closest point, it returns -1

    return Point(closestPtX, closestPtY);
  }

	//constructor
	searchLINK(string expt_f_name, string expt_name, string out_folder_name, LINK startL, LINK goalL, int modeInput){

		expt_fName = expt_f_name;
		expt_Name = expt_name;
		out_folderName = out_folder_name;
		expt_folderName = expt_fName.substr(0, expt_fName.find_last_of("/\\")+1);

		flagmin = -1;
    n_successors = 0;
    SAVE_FLAG = 0;

		cout << "start reading RSJresource" << endl;
		// Read from file
		ifstream my_fstream (expt_fName);
		expt_container = RSJresource (my_fstream)[expt_Name];
		map_image_fName = expt_folderName + expt_container["map_name"].as<string>();
		//grey_map = cvParseMap2d(map_image_fName, false); // computes representative points

		cout << "map loading complete" << endl;

		// read data for planning
    original_map = imread(map_image_fName, CV_LOAD_IMAGE_COLOR);
    MIN_X = 0;
		MAX_X = original_map.cols;
		MIN_Y = 0;
		MAX_Y = original_map.rows;

    this->startLink = startL;
    this->goalLink = goalL;
    searchMode = modeInput;

		cout << "reading complete" << endl;
    cout << "Search Mode: " << searchMode << endl;

    cvtColor(original_map, grey_map, CV_RGB2GRAY);
    if (searchMode == 0){
      threshold(grey_map, grey_map, 80, 255, 0);
    }else{
      threshold(grey_map, grey_map, 20, 255, 0);
    }
    cvtColor(grey_map, color_map, CV_GRAY2RGB);
    resize(color_map, realtime_display, Size(), PLOT_SCALE, PLOT_SCALE, INTER_NEAREST);

    if (searchMode == 0){
      printf("Pre-calculating for heuristic function...");

      struct timespec start_time, finish_time;
      clock_gettime(CLOCK_MONOTONIC, &start_time);

      Mat pre_result;
      int Mapping_scalar = 4;
      resize(color_map, pre_result, Size(), Mapping_scalar, Mapping_scalar, INTER_NEAREST);

      SkeletonMat = new Point * [MAX_Y];
      for(int j = 0; j < MAX_Y; j++){
        SkeletonMat[j] = new Point [MAX_X];
        for(int i = 0; i < MAX_X; i++){
          if (original_map.at<Vec3b>(j, i).val[0] <= OBSTHRESHOLD){
            SkeletonMat[j][i] = Point(-1, -1);
          }else{
            SkeletonMat[j][i] = PresetClosestPt(Point(i,j), color_map);
            if (SkeletonMat[j][i] != Point(-1,-1)){
              line(pre_result, cvPoint(i * Mapping_scalar, j * Mapping_scalar),
                cvPoint(sk_cost[SkeletonMat[j][i].x].x * Mapping_scalar, sk_cost[SkeletonMat[j][i].x].y * Mapping_scalar), CV_RGB(200, 200, 200), LINE_THICKNESS);
            }else{
              SkeletonMat[j][i] = Point(0, 0);
            }
          }
        }
      }
      double d;
      clock_gettime(CLOCK_MONOTONIC, &finish_time);
      d = (finish_time.tv_sec - start_time.tv_sec);
      d += (finish_time.tv_nsec - start_time.tv_nsec) / 1000000000.0;

      // plot the skeleton
      for(auto i : sk_cost){
        cvPlotPoint(pre_result, cv_plot_coord(i.x * Mapping_scalar / PLOT_SCALE, i.y * Mapping_scalar / PLOT_SCALE), CV_RGB(0, 165, 81), VERTEX_SIZE * 2);
      }
      time_t now = time(0);
	    tm* localtm = localtime(&now);
      printf("completed! Pre-cal time: %g\n", d);
      imshow("Precalculation", pre_result);
      imwrite("outfiles/"
              + to_string(1900 + localtm->tm_year) + "_"
              + to_string(localtm->tm_mon) + "_"
              + to_string(localtm->tm_mday) + "_"
              + to_string(localtm->tm_hour) + "_"
              + to_string(localtm->tm_min) + "_shortest_search.png", pre_result);
      cvWaitKey();
      destroyWindow("Precalculation");
    }

    //realtime_display = color_map.clone();
    //threshold(realtime_display, realtime_display, 200, 255, 0);
		//resize(realtime_display, realtime_display, Size(), PLOT_SCALE, PLOT_SCALE, INTER_NEAREST);

		// plot start link and goal point
		cvPlotPoint(realtime_display, cv_plot_coord(startLink.x, startLink.y), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);
		cvPlotPoint(realtime_display, cv_plot_coord(goalLink.x, goalLink.y), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);

		//namedWindow( "Display window", WINDOW_AUTOSIZE);
		//imshow("Display window", realtime_display);
		//waitKey(0);
    line(realtime_display, cv_plot_coord(startLink.x, startLink.y),
			cv_plot_coord((startLink.x + (startLink.expansion * L + AA * 2) * cos(startLink.theta)), (startLink.y + (startLink.expansion * L + AA * 2) * sin(startLink.theta))), CV_RGB(200, 200, 200), LINE_THICKNESS);

		start_time = clock();
	}

  bool InterOnSeg(POINT intersectP, SEG seg, bool m) {
    //for checking if intersection point is on the segment
    //point on any end of the segment => NOT on the segment


    //m = false: normal mode, check if the point is on the segment
    //m = true; on-ends mode, point on either of ends of segment counted as on the segment
    if (seg.pt1.x < seg.pt2.x) {
      if (seg.pt1.x < intersectP.x && intersectP.x < seg.pt2.x) {
        return true;
      }else if (m){
        if (seg.pt1.x == intersectP.x || intersectP.x == seg.pt2.x)
          return true;
      }
    }
    else if (seg.pt2.x < seg.pt1.x) {
      if (seg.pt2.x < intersectP.x && intersectP.x < seg.pt1.x) {
        return true;
      }else if (m){
        if (seg.pt1.x == intersectP.x || intersectP.x == seg.pt2.x)
          return true;
      }
    }
    else if (seg.pt1.y < seg.pt2.y) {//if the segment is vertical
      if (seg.pt1.y < intersectP.y && intersectP.y < seg.pt2.y) {
        return true;
      }else if (m){
        if (seg.pt1.y == intersectP.y || intersectP.y == seg.pt2.y)
          return true;
      }
    }
    else if (seg.pt2.y < seg.pt1.y) {
      if (seg.pt2.y < intersectP.y && intersectP.y < seg.pt1.y) {
        return true;
      }else if (m){
        if (seg.pt1.y == intersectP.y || intersectP.y == seg.pt2.y)
          return true;
      }
    }
    else {
      printf("Error! The points constructing the segment overlap! POINT1:(%2lf, %2lf) POINT2:(%2lf, %2lf)\n", seg.pt1.x, seg.pt1.y, seg.pt2.x, seg.pt2.y);
    }

    return false;

  }

  int CheckPtStatus(SEG s, POINT p, bool CheckOnSeg) {
    //chech a point is on which side of the reference segment
    //if CheckOnSeg == true, return 2 when the point is on the segment(not including on both ends), -2 for not on
    //if CheckOnSeg == false, return 0 and don't check if the point is on the segment

    //left and right hand sides of the line equation
    double LHS = (s.pt2.x - s.pt1.x) * (p.y - s.pt1.y);
    double RHS = (s.pt2.y - s.pt1.y) * (p.x - s.pt1.x);
    //on one side
    if (LHS > RHS)
      return 1;
    //on the other side
    if (LHS < RHS)
      return -1;
    //point is on the line
    if (LHS == RHS) {
      if (CheckOnSeg) {
        if (InterOnSeg(p, s, true)) {
          //if on the segment
          return 2;
        }
        else {
          //if not
          return -2;
        }
      }
      else {
        //not care about if the position of the point is on the segment
        return 0;
      }
    }
  }

  bool CheckSegIntersect(SEG seg1, SEG seg2) {
    //check if there's an intersection of 2 segments
    //m = false, not including both ends of segments
    //m = true, including both ends


    double x, y;
    //dominator
    double dom = ((seg1.pt2.y - seg1.pt1.y) * (seg2.pt2.x - seg2.pt1.x) - (seg2.pt2.y - seg2.pt1.y) * (seg1.pt2.x - seg1.pt1.x));

    //if parallel
    if (dom == 0) {
      //if overlap, it is considered as an intersection
      if (CheckPtStatus(seg1, seg2.pt1, true) == 2 || CheckPtStatus(seg1, seg2.pt2, true) == 2 ||
        CheckPtStatus(seg2, seg1.pt1, true) == 2 || CheckPtStatus(seg2, seg1.pt2, true) == 2) {
        return true;
      }
      else if (((seg1.pt1.x == seg2.pt1.x && seg1.pt1.y == seg2.pt1.y) && (seg1.pt2.x == seg2.pt2.x && seg1.pt2.y == seg2.pt2.y)) ||
        ((seg1.pt1.x == seg2.pt2.x && seg1.pt1.y == seg2.pt2.y) && (seg1.pt2.x == seg2.pt1.x && seg1.pt2.y == seg2.pt1.y))){
        //if identical with the other segment
        return true;
      }
      //if not overlap
      return false;
    }

    //if not parallel
    //coordinates of intersection point
    x = (((seg1.pt2.y - seg1.pt1.y) * seg1.pt1.x - (seg1.pt2.x - seg1.pt1.x) * seg1.pt1.y) * (seg2.pt2.x - seg2.pt1.x)
      - ((seg2.pt2.y - seg2.pt1.y) * seg2.pt1.x - (seg2.pt2.x - seg2.pt1.x) * seg2.pt1.y) * (seg1.pt2.x - seg1.pt1.x))
      / dom;

    y = (((seg2.pt2.y - seg2.pt1.y) * seg2.pt1.x - (seg2.pt2.x - seg2.pt1.x) * seg2.pt1.y) * (seg1.pt2.y - seg1.pt1.y)
      - ((seg1.pt2.y - seg1.pt1.y) * seg1.pt1.x - (seg1.pt2.x - seg1.pt1.x) * seg1.pt1.y) * (seg2.pt2.y - seg2.pt1.y))
      / (-dom);


    //check if intersection point is not on any extension of segments
    if (InterOnSeg(POINT(x, y), seg1, true) && InterOnSeg(POINT(x, y), seg2, true)){
      return true;
    }
    return false;
  }

  bool ChkPthSelf_tangle(vector<LINK> inputPath){

    vector<POINT> path;
    for(auto it : inputPath){
      path.push_back(it.getPoint());
    }
    path.push_back(inputPath.back().getHead());

    vector<int> h, hu, t;
    POINT thisPta, lastPta, thisPtb, lastPtb;

    //check if it crosses itself
    thisPta = path.front();
    for(int a = 1; a < (path.size() - 2); a++){
      lastPta = thisPta;
      thisPta = path[a];

      thisPtb = path[a + 1];
      for(int b = a + 2; b < path.size(); b++){
        lastPtb = thisPtb;
        thisPtb = path[b];
        if (CheckSegIntersect(SEG(lastPta, thisPta), SEG(lastPtb, thisPtb))){
          return true;
        }
      }
    }

    return false;
  }

  void getClosestPt(LINK & n){ // get the closest point of sk_cost/sk_deu to the end of this link
    int rdY = min(MAX_Y - 1, max(MIN_Y, (int)round(n.getHeadY())));
    int rdX = min(MAX_X - 1, max(MIN_X, (int)round(n.getHeadX())));
    if ((int)round(n.getHeadX()) < MIN_X){
      cout << "exceed low X bound: " << (int)round(n.getHeadX()) << ", corrected: " << rdX << endl;
    }
    if ((int)round(n.getHeadY()) < MIN_Y){
      cout << "exceed low Y bound: " << (int)round(n.getHeadY()) << ", corrected: " << rdY << endl;
    }
    if ((int)round(n.getHeadX()) >= MAX_X){
      cout << "exceed upper X bound: " << (int)round(n.getHeadX()) << ", corrected: " << rdX << endl;
    }
    if ((int)round(n.getHeadY()) >= MAX_Y){
      cout << "exceed upper Y bound: " << (int)round(n.getHeadY()) << ", corrected: " << rdY << endl;
    }
    Point ptIndex = SkeletonMat[rdY][rdX];
    n.iClosestP_cost = ptIndex.x;
    n.iClosestP_heu = ptIndex.y;
    // if (n.iClosestP_cost != -1 && n.iClosestP_heu != -1)
    //   printf("closest cost point: %d, closest heu point: %d\n", n.iClosestP_cost, n.iClosestP_heu);
  }

  int getClosestIndex(POINT n){ // get the closest point of sk_cost/sk_deu to the end of this link
    int rdY = min(MAX_Y - 1, max(MIN_Y, (int)round(n.y)));
    int rdX = min(MAX_X - 1, max(MIN_X, (int)round(n.x)));
    if ((int)round(n.x) < MIN_X){
      cout << "exceed low X bound: " << (int)round(n.x) << ", corrected: " << rdX << endl;
    }
    if ((int)round(n.y) < MIN_Y){
      cout << "exceed low Y bound: " << (int)round(n.y) << ", corrected: " << rdY << endl;
    }
    if ((int)round(n.x) >= MAX_X){
      cout << "exceed upper X bound: " << (int)round(n.x) << ", corrected: " << rdX << endl;
    }
    if ((int)round(n.y) >= MAX_Y){
      cout << "exceed upper Y bound: " << (int)round(n.y) << ", corrected: " << rdY << endl;
    }
    return SkeletonMat[rdY][rdX].x;
  }

  void forceDetector(LINK & n){//check reaction forces
    //Fx, Fy, x, y, scalar, weight (for combining multiple consecutive forces in the same direction)
    double range = 1.5;
    double first_left = -1, first_right = -1, last_left = -1, last_right = -1;
    //check if any pixel beside the link is obsessed by wall
    for(double i = 3; i < (n.expansion * L + AA * 2 - 3); i++){
			if (grey_map.at<uchar>((int)round(n.y + i * sin(n.theta) + range * sin(n.theta - 0.5 * PI)),
                            (int)round(n.x + i * cos(n.theta) + range * cos(n.theta - 0.5 * PI))) <= OBSTHRESHOLD){//left side
        if (first_left == -1) first_left = i;//record the first touch
        if (last_left < i) last_left = i;
      }
      if (grey_map.at<uchar>((int)round(n.y + i * sin(n.theta) + range * sin(n.theta + 0.5 * PI)),
                            (int)round(n.x + i * cos(n.theta) + range * cos(n.theta + 0.5 * PI))) <= OBSTHRESHOLD){//right side
        if (first_right == -1) first_right = i;
        if (last_right < i) last_right = i;
      }
		}

    //record force on the left
    if(first_left != -1 && last_left != -1 && ((last_left - first_left) > (n.expansion * L / 2 + AA - 3))){
      double last_x = 0, last_y = 0, last_weight = 0, last_s_n = 2, last_s_f = 2;
      double new_x = n.x + (first_left + last_left) / 2 * cos(n.theta) + range * cos(n.theta - 0.5 * PI);
      double new_y = n.y + (first_left + last_left) / 2 * sin(n.theta) + range * sin(n.theta - 0.5 * PI);
      if (!n.forces.empty()){ //check if it has forces record
        if (fabs(n.forces.rbegin()[1][0] - cos(n.theta + 0.5 * PI)) < INFINITESIMAL_DOUBLE && fabs(n.forces.rbegin()[1][1] - sin(n.theta + 0.5 * PI)) < INFINITESIMAL_DOUBLE
          && fabs((new_x - n.forces.rbegin()[1][2]) * sin(n.theta) - (new_y - n.forces.rbegin()[1][3]) * cos(n.theta)) < INFINITESIMAL_DOUBLE){
          // if the last normal force has the same direction with the new one and whether the wire between two touching points is parallel with the new link
          last_x = n.forces.back()[2];
          last_y = n.forces.back()[3];
          last_s_n = n.forces.rbegin()[1][4];
          last_s_f = n.forces.back()[4];
          last_weight = n.forces.back()[5];
          n.forces.pop_back();
          n.forces.pop_back();
          //cout << "found same force on the left" << endl;
        }
      }
      vector<double> f;
      f.push_back(cos(n.theta + 0.5 * PI)); // fx
      f.push_back(sin(n.theta + 0.5 * PI)); // fy
      f.push_back((new_x + last_x * last_weight) / (1 + last_weight)); // x of the touching point
      f.push_back((new_y + last_y * last_weight) / (1 + last_weight)); // y of the touching point
      f.push_back(last_s_n); // Scalar of force
      f.push_back(1.0 + last_weight); // weight
      n.forces.push_back(f); // normal force

      // adjoint static friction, pointing drillpoint
      f[0] = cos(n.theta - (cos(n.theta) * cos(drillLink.theta) + sin(n.theta) * sin(drillLink.theta) < 0 ? PI : 0.0)); // change the direction for friction
      f[1] = sin(n.theta - (cos(n.theta) * cos(drillLink.theta) + sin(n.theta) * sin(drillLink.theta) < 0 ? PI : 0.0)); // change the direction for friction
      f[4] = 1.0 * last_s_f;
      n.forces.push_back(f);

      n.brace = true;
    }else{
      n.brace = false;
    }
    //record force on the right
    if(first_right != -1 && last_right != -1 && ((last_right - first_right) > (n.expansion * L / 2 + AA - 3))){
      double last_x = 0, last_y = 0, last_weight = 0, last_s_n = 2, last_s_f = 2;
      double new_x = n.x + (first_right + last_right) / 2 * cos(n.theta) + range * cos(n.theta + 0.5 * PI);
      double new_y = n.y + (first_right + last_right) / 2 * sin(n.theta) + range * sin(n.theta + 0.5 * PI);
      if (!n.forces.empty()){
        if (fabs(n.forces.rbegin()[1][0] - cos(n.theta - 0.5 * PI)) < INFINITESIMAL_DOUBLE && fabs(n.forces.rbegin()[1][1] - sin(n.theta - 0.5 * PI)) < INFINITESIMAL_DOUBLE
          && fabs((new_x - n.forces.rbegin()[1][2]) * sin(n.theta) - (new_y - n.forces.rbegin()[1][3]) * cos(n.theta)) < INFINITESIMAL_DOUBLE){
          last_x = n.forces.back()[2];
          last_y = n.forces.back()[3];
          last_s_n = n.forces.rbegin()[1][4];
          last_s_f = n.forces.back()[4];
          last_weight = n.forces.back()[5];
          n.forces.pop_back();
          n.forces.pop_back();
          //cout << "found same force on the right" << endl;
        }
      }
      vector<double> f;
      f.push_back(cos(n.theta - 0.5 * PI));//fx
      f.push_back(sin(n.theta - 0.5 * PI));//fy
      f.push_back((new_x + last_x * last_weight) / (1 + last_weight));//x of the touching point
      f.push_back((new_y + last_y * last_weight) / (1 + last_weight));//y of the touching point
      f.push_back(last_s_n);  // Scalar
      f.push_back(1.0 + last_weight); // weight
      n.forces.push_back(f); // normal force

      // adjoint static friction, pointing drillpoint
      f[0] = cos(n.theta - (cos(n.theta) * cos(drillLink.theta) + sin(n.theta) * sin(drillLink.theta) < 0 ? PI : 0.0));
      f[1] = sin(n.theta - (cos(n.theta) * cos(drillLink.theta) + sin(n.theta) * sin(drillLink.theta) < 0 ? PI : 0.0));
      f[4] = last_s_f;
      n.forces.push_back(f);

      n.brace = true;
    }
  }

  bool forceSpanningCheck(LINK & n, bool D_F){ // D_F == true, DRILLFORCE switch on
    // there must exists even number of forces
    if (n.forces.empty() || n.forces.size() % 2 != 0) return false;

    // define the direction of gravity
    GRAVITYDIR;

    if(D_F){
      // Reconstruct the path with the this link
      vector<LINK *> p = reconstructPointerPath(n);
      vector<LINK> pathLks;
      for (int i = p.size() - 1; i >= 0; i--)
        pathLks.push_back(* p[i]);

      int nV = n.forces.size(), nC = n.forces.size() + 3;
      // nV = number of variables, nC = number of constraints

      real_t HM[nV * nV];
      real_t GM[nV];
      real_t AM[nC * nV];
      real_t lb[nV];
      real_t ub[nV];
      real_t lbA[nC];
      real_t ubA[nC];

      // initialize HM and GM
      for(int_t i = 0; i < nV; i++ ){
        for(int_t j = 0; j < nV; j++ ){
          HM[i * nV + j] = 0.0;
        }
        GM[i] = 0.0;
      }


      if (!n.forces.empty()){
        double constTerm = n.gAccT + ((drillLink.x - n.getHeadX()) * sin(drillLink.theta + PI) - (drillLink.y - n.getHeadY()) * cos(drillLink.theta + PI)) * DRILLFORCE * P2M_SCALE;
        vector<double> coeF;
        for(int i = 0; i < n.forces.size(); i++){ // calculate the coefficients of forces
          coeF.push_back(((n.forces[i][2] - n.getHeadX()) * n.forces[i][1] - (n.forces[i][3] - n.getHeadY()) * n.forces[i][0]) * P2M_SCALE);
        }
        for(int i = 0; i < n.forces.size(); i++){
          GM[i] += 2 * constTerm * coeF[i];
          for(int j = 0; j < n.forces.size(); j++){
            HM[i * nV + j] += 2 * coeF[i] * coeF[j];
          }
        }
      }


/*
      for(auto a : pathLks){
        if (!a.forces.empty()){
          double constTerm = a.gAccT + ((drillLink.x - a.getHeadX()) * sin(drillLink.theta + PI) - (drillLink.y - a.getHeadY()) * cos(drillLink.theta + PI)) * DRILLFORCE * P2M_SCALE;
          vector<double> coeF;
          for(int i = 0; i < a.forces.size(); i++){ // calculate the coefficients of forces
            coeF.push_back(((a.forces[i][2] - a.getHeadX()) * a.forces[i][1] - (a.forces[i][3] - a.getHeadY()) * a.forces[i][0]) * P2M_SCALE);
          }
          for(int i = 0; i < a.forces.size(); i++){
            GM[i] += 2 * constTerm * coeF[i];
            for(int j = 0; j < a.forces.size(); j++){
              HM[i * nV + j] += 2 * coeF[i] * coeF[j];
            }
          }
        }
      }
*/

      for(int_t j = 0; j < nV; j++){
        for(int_t i = 0; i < nC; i++){
          if (i < 2){
            AM[i * nV + j] = fabs(n.forces[(int)j][(int)i]) < INFINITESIMAL_DOUBLE? 0.0 : n.forces[(int)j][(int)i];
          }else if (i == 2){
            AM[i * nV + j] = ((n.forces[(int)j][2] - n.getHeadX()) * n.forces[(int)j][1] - (n.forces[(int)j][3] - n.getHeadY()) * n.forces[(int)j][0]) * P2M_SCALE;
          }else if (i == ((int_t)(j / 2) + 3)){
            AM[i * nV + j] = ((int)j % 2 == 0)? -MU : 1.0;
          }else if (i == ((int)(j / 2) + 3 + (int)(n.forces.size() / 2))){
            AM[i * nV + j] = ((int)j % 2 == 0)? MU : 1.0;
          }else{
            AM[i * nV + j] = 0.0;
          }
        }
      }

      for(int_t i = 0; i < nV; i++ ){
        lb[i] = (i % 2 == 0)? 0.0 : -MU * MAXFORCE;
        ub[i] = ((i % 2 == 0)? 1.0 : MU) * MAXFORCE;
      }

      for(int_t i = 0; i < nC; i++){
        if (i == 0){
          lbA[i] = -n.gAccX - cos(drillLink.theta + PI) * DRILLFORCE;
          ubA[i] = -n.gAccX - cos(drillLink.theta + PI) * DRILLFORCE;
        }else if (i == 1){
          lbA[i] = -n.gAccY - sin(drillLink.theta + PI) * DRILLFORCE;
          ubA[i] = -n.gAccY - sin(drillLink.theta + PI) * DRILLFORCE;
        }else if (i == 2){
          lbA[i] = -n.gAccT - ((drillLink.x - n.getHeadX()) * sin(drillLink.theta + PI) - (drillLink.y - n.getHeadY()) * cos(drillLink.theta + PI)) * DRILLFORCE * P2M_SCALE;
          ubA[i] = -n.gAccT - ((drillLink.x - n.getHeadX()) * sin(drillLink.theta + PI) - (drillLink.y - n.getHeadY()) * cos(drillLink.theta + PI)) * DRILLFORCE * P2M_SCALE;
        }else if (i < (n.forces.size() / 2 + 3)){
          lbA[i] = - 2 * MU * MAXFORCE;
          ubA[i] = 0.0;
        }else{
          lbA[i] = 0.0;
          ubA[i] = 2 * MU * MAXFORCE;
        }
      }

      QProblem exampleCP(nV, nC);

      exampleCP.setPrintLevel(PL_NONE);
      int_t nWSR = 100;

      if (SUCCESSFUL_RETURN == exampleCP.init(HM, GM, AM, lb, ub, lbA, ubA, nWSR)){
        printf("Able to balance drill force: (%g, %g), gravity: (%g, %g) and moment of gravity: %g\n", DRILLFORCE * cos(drillLink.theta + PI), DRILLFORCE * sin(drillLink.theta + PI), n.gAccX, n.gAccY, n.gAccT);
        real_t xOpt[nV];
        exampleCP.getPrimalSolution(xOpt);
        for(int i = 0; i < nV; i++){
          n.forces[i][4] = xOpt[i];
          printf("Force %s %d, primal scalar %g\n", ((i % 2 == 0)? "(N)": "(F)"), (int)(i / 2), xOpt[i]);
        }
        return true;
      }else{
        return false;
      }

    }else{
      // tapping
      /*
      int nV = n.forces.size(), nC = n.forces.size() + 3;
      // nV = number of variables, nC = number of constraints

      real_t HM[nV * nV];
      real_t AM[nC * nV];
      real_t GM[nV];
      real_t lb[nV];
      real_t ub[nV];
      real_t lbA[nC];
      real_t ubA[nC];

      double constTerm = n.gAccT;
      vector<double> coeF;
      for(int i = 0; i < nV; i++){ // calculate the coefficients of forces
        coeF.push_back(((n.forces[i][2] - n.getHeadX()) * n.forces[i][1] - (n.forces[i][3] - n.getHeadY()) * n.forces[i][0]) * P2M_SCALE);
      }
      for(int i = 0; i < nV; i++){
        GM[i] = 2 * constTerm * coeF[i];
        for(int j = 0; j < nV; j++){
          HM[i * nV + j] = 2 * coeF[i] * coeF[j];
        }
      }

      for(int_t j = 0; j < nV; j++){
        for(int_t i = 0; i < nC; i++){
          if (i < 2){
            AM[i * nV + j] = fabs(n.forces[(int)j][(int)i]) < INFINITESIMAL_DOUBLE? 0.0 : n.forces[(int)j][(int)i];
          }else if (i == 2){
            AM[i * nV + j] = ((n.forces[(int)j][2] - n.getHeadX()) * n.forces[(int)j][1] - (n.forces[(int)j][3] - n.getHeadY()) * n.forces[(int)j][0]) * P2M_SCALE;
          }else if (i == ((int_t)(j / 2) + 3)){
            AM[i * nV + j] = ((int)j % 2 == 0)? -MU : 1.0;
          }else if (i == ((int)(j / 2) + 3 + (int)(n.forces.size() / 2))){
            AM[i * nV + j] = ((int)j % 2 == 0)? MU : 1.0;
          }else{
            AM[i * nV + j] = 0.0;
          }
        }
      }

      for(int_t i = 0; i < nV; i++ ){
        lb[i] = (i % 2 == 0)? 0.0 : -MU * MAXFORCE;
        ub[i] = ((i % 2 == 0)? 1.0 : MU) * MAXFORCE;
      }

      for(int_t i = 0; i < nC; i++){
        if (i == 0){
          lbA[i] = -n.gAccX;
          ubA[i] = -n.gAccX;
        }else if (i == 1){
          lbA[i] = -n.gAccY;
          ubA[i] = -n.gAccY;
        }else if (i == 2){
          lbA[i] = -n.gAccT;
          ubA[i] = -n.gAccT;
        }else if (i < (n.forces.size() / 2 + 3)){
          lbA[i] = - 2 * MU * MAXFORCE;
          ubA[i] = 0.0;
        }else{
          lbA[i] = 0.0;
          ubA[i] = 2 * MU * MAXFORCE;
        }
      }
      */
      int nV = n.forces.size() + 1, nC = n.forces.size() + 3;
      // nV = number of variables, nC = number of constraints

      real_t HM[nV * nV];
      real_t AM[nC * nV];
      real_t GM[nV];
      real_t lb[nV];
      real_t ub[nV];
      real_t lbA[nC];
      real_t ubA[nC];

      for(int_t i = 0; i < nV; i++ ){
        for(int_t j = 0; j < nV; j++ ){
          if (i == (nV - 1) && j == (nV - 1)){
            HM[i * nV + j] = 1.0; // for the torque acting at the end of the link
          }else{
            HM[i * nV + j] = 0.0;
          }
        }
      }

      for(int_t i = 0; i < nV; i++ ){
        GM[i] = 0.0;
      }

      for(int_t j = 0; j < nV; j++){
        for(int_t i = 0; i < nC; i++){
          if(j < n.forces.size()){
            if (i < 2){
              AM[i * nV + j] = fabs(n.forces[(int)j][(int)i]) < INFINITESIMAL_DOUBLE? 0.0 : n.forces[(int)j][(int)i];
            }else if (i == 2){
              AM[i * nV + j] = ((n.forces[(int)j][2] - n.getHeadX()) * n.forces[(int)j][1] - (n.forces[(int)j][3] - n.getHeadY()) * n.forces[(int)j][0]) * P2M_SCALE;
            }else if (i == ((int_t)(j / 2) + 3)){
              AM[i * nV + j] = ((int)j % 2 == 0)? -MU : 1.0;
            }else if (i == ((int)(j / 2) + 3 + (int)(n.forces.size() / 2))){
              AM[i * nV + j] = ((int)j % 2 == 0)? MU : 1.0;
            }else{
              AM[i * nV + j] = 0.0;
            }
          }else{ // torque acting on the end
            if (i == 2){
              AM[i * nV + j] = 1.0;
            }else{
              AM[i * nV + j] = 0.0;
            }
          }
        }
      }

      for(int_t i = 0; i < nV; i++ ){
        if (i < n.forces.size()){
          lb[i] = (i % 2 == 0)? 0 : -MU * MAXFORCE;
          ub[i] = ((i % 2 == 0)? 1 : MU) * MAXFORCE;
        }else{
          lb[i] = -MAXTORQUE;
          ub[i] = MAXTORQUE;
        }
      }

      for(int_t i = 0; i < nC; i++){
        if (i == 0){
          lbA[i] = -n.gAccX;
          ubA[i] = -n.gAccX;
        }else if (i == 1){
          lbA[i] = -n.gAccY;
          ubA[i] = -n.gAccY;
        }else if (i == 2){
          lbA[i] = -n.gAccT;
          ubA[i] = -n.gAccT;
        }else if (i < (n.forces.size() / 2 + 3)){
          lbA[i] = - 2 * MU * MAXFORCE;
          ubA[i] = 0.0;
        }else{
          lbA[i] = 0.0;
          ubA[i] = 2 * MU * MAXFORCE;
        }
      }

      QProblem exampleCP(nV, nC);

      exampleCP.setPrintLevel(PL_NONE);
      int_t nWSR = 100;

      if (SUCCESSFUL_RETURN == exampleCP.init(HM, GM, AM, lb, ub, lbA, ubA, nWSR)){
        // printf("Able to balance gravity of robot body: (%g, %g) and moment of gravity: %g\n", n.gAccX, n.gAccY, n.gAccT);

        // cout << endl;
        // cout << "HM =" << endl;
        // for(int i = 0; i < nV; i++){
        //   for(int j = 0; j < nV; j++){
        //     cout << HM[i * nV + j] << "\t";
        //   }
        //   cout << endl;
        // }

        // cout << endl;
        // cout << "AM =" << endl;
        // for(int i = 0; i < nC; i++){
        //   for(int j = 0; j < nV; j++){
        //     cout << AM[i * nV + j] << "\t";
        //   }
        //   cout << endl;
        // }

        // cout << endl;
        // cout << "GM =" << endl;
        // for(int j = 0; j < nV; j++){
        //   cout << GM[j] << "\t";
        // }
        // cout << endl;

        // cout << endl;
        // cout << "lb =" << endl;
        // for(int j = 0; j < nV; j++){
        //   cout << lb[j] << "\t";
        // }
        // cout << endl;

        // cout << endl;
        // cout << "ub =" << endl;
        // for(int j = 0; j < nV; j++){
        //   cout << ub[j] << "\t";
        // }
        // cout << endl;

        // cout << endl;
        // cout << "lbA =" << endl;
        // for(int j = 0; j < nC; j++){
        //   cout << lbA[j] << "\t";
        // }
        // cout << endl;

        // cout << endl;
        // cout << "ubA =" << endl;
        // for(int j = 0; j < nC; j++){
        //   cout << ubA[j] << "\t";
        // }
        // cout << endl;

        return true;
      }else{
        return false;
      }
    }
  }

  double torqueCheck(LINK & n){ // torque about the end of path
    // there must exists even number of forces
    if (n.forces.size() % 2 != 0) return false;

    // define the direction of gravity
    GRAVITYDIR;

    int nV = n.forces.size() + 3, nC = n.forces.size() + 3;
    // nV = number of variables, nC = number of constraints

    real_t HM[nV * nV];
    real_t AM[nC * nV];
    real_t GM[nV];
    real_t lb[nV];
    real_t ub[nV];
    real_t lbA[nC];
    real_t ubA[nC];

    for(int_t i = 0; i < nV; i++ ){
      for(int_t j = 0; j < nV; j++ ){
        if (i == (n.forces.size() + 2) && j == (n.forces.size() + 2)){
          HM[i * nV + j] = 1.0; // for the torque acting at the end of the link
        }else{
          HM[i * nV + j] = 0.0;
        }
      }
    }

    for(int_t j = 0; j < nV; j++){
      for(int_t i = 0; i < nC; i++){
        if(j < n.forces.size()){
          if (i < 2){
            AM[i * nV + j] = fabs(n.forces[(int)j][(int)i]) < INFINITESIMAL_DOUBLE? 0.0 : n.forces[(int)j][(int)i];
          }else if (i == 2){
            AM[i * nV + j] = ((n.forces[(int)j][2] - n.getHeadX()) * n.forces[(int)j][1] - (n.forces[(int)j][3] - n.getHeadY()) * n.forces[(int)j][0]) * P2M_SCALE;
          }else if (i == ((int_t)(j / 2) + 3)){
            AM[i * nV + j] = ((int)j % 2 == 0)? -MU : 1.0;
          }else if (i == ((int)(j / 2) + 3 + (int)(n.forces.size() / 2))){
            AM[i * nV + j] = ((int)j % 2 == 0)? MU : 1.0;
          }else{
            AM[i * nV + j] = 0.0;
          }
        }else if (j == n.forces.size()){ // x force acting on the end
          if (i == 0){
            AM[i * nV + j] = 1.0;
          }else{
            AM[i * nV + j] = 0.0;
          }
        }else if (j == (n.forces.size() + 1)){ // y force acting on the end
          if (i == 1){
            AM[i * nV + j] = 1.0;
          }else{
            AM[i * nV + j] = 0.0;
          }
        }else if (j == (n.forces.size() + 2)){ // torque acting on the end
          if (i == 2){
            AM[i * nV + j] = 1.0;
          }else{
            AM[i * nV + j] = 0.0;
          }
        }
      }
    }

    for(int_t i = 0; i < nV; i++ ){
      GM[i] = 0.0;
    }

    for(int_t i = 0; i < nV; i++ ){
      if (i < n.forces.size()){
        lb[i] = (i % 2 == 0)? 0 : -MU * MAXFORCE;
        ub[i] = ((i % 2 == 0)? 1 : MU) * MAXFORCE;
      }else if (i < (n.forces.size() + 2)){
        lb[i] = -MAXFORCE / sqrt(2);
        ub[i] = MAXFORCE / sqrt(2);
      }else{
        lb[i] = -INFTY;
        ub[i] = INFTY;
      }
    }

    for(int_t i = 0; i < nC; i++){
      if (i == 0){
        lbA[i] = -n.gAccX - cos(drillLink.theta + PI) * DRILLFORCE;
        ubA[i] = -n.gAccX - cos(drillLink.theta + PI) * DRILLFORCE;
      }else if (i == 1){
        lbA[i] = -n.gAccY - sin(drillLink.theta + PI) * DRILLFORCE;
        ubA[i] = -n.gAccY - sin(drillLink.theta + PI) * DRILLFORCE;
      }else if (i == 2){
        lbA[i] = -n.gAccT - ((drillLink.x - n.getHeadX()) * sin(drillLink.theta + PI) - (drillLink.y - n.getHeadY()) * cos(drillLink.theta + PI)) * DRILLFORCE * P2M_SCALE;
        ubA[i] = -n.gAccT - ((drillLink.x - n.getHeadX()) * sin(drillLink.theta + PI) - (drillLink.y - n.getHeadY()) * cos(drillLink.theta + PI)) * DRILLFORCE * P2M_SCALE;
      }else if (i < (n.forces.size() / 2 + 3)){
        lbA[i] = - 2 * MU * MAXFORCE;
        ubA[i] = 0.0;
      }else{
        lbA[i] = 0.0;
        ubA[i] = 2 * MU * MAXFORCE;
      }
    }

    QProblem exampleCP(nV, nC);

    exampleCP.setPrintLevel(PL_NONE);
    int_t nWSR = 100;

    if (SUCCESSFUL_RETURN == exampleCP.init(HM, GM, AM, lb, ub, lbA, ubA, nWSR)){
      double maxtor = sqrt(fabs(exampleCP.getObjVal()) * 2);
      if(maxtor > MAXTORQUE){
        //cout << "Exceeded joint torque limit: " << maxtor << endl;
        return -1.0;
      }else{
        // cout << "Joint torque within limit: " << (maxtor < INFINITESIMAL_DOUBLE? 0 : maxtor) << endl;
        return maxtor;
      }
    }else{
      //cout << "Torque checking: cannot balance!" << endl;
      return -1.0;
    }
  }

	bool isLinkAccessible (const LINK & tn, LINK & ol) {//ol = original LINK
    if (grey_map.at<uchar>((int)round(tn.getHeadY()), (int)round(tn.getHeadX())) <= OBSTHRESHOLD) return false;
    // is the link in workspace
    if (tn.getHeadX() < MIN_X || tn.getHeadX() > MAX_X || tn.getHeadY() < MIN_Y || tn.getHeadY() > MAX_Y)
      return false;
		//check each point on the link, interval = 1, and the end as well
    for(double i = tn.getLength(); i > 0.0; ){
			if (grey_map.at<uchar>((int)round(tn.y + i * sin(tn.theta)), (int)round(tn.x + i * cos(tn.theta))) <= OBSTHRESHOLD) return false;
      i -= 0.5;
		}

    if (ol != startLink){
  		//check if the passage(shortcut) is accessible
  		double y_d = tn.y + (tn.expansion * L + AA * 2) * sin(tn.theta) - ol.y, x_d = tn.x + (tn.expansion * L + AA * 2) * cos(tn.theta) - ol.x;
  		double new_theta = atan2(y_d, x_d);
  		for(double i = 1; i <= sqrt(pow(y_d, 2) + pow(x_d, 2)); i+=1){
  			if (grey_map.at<uchar>((int)round(ol.y + i * sin(new_theta)), (int)round(ol.x + i * cos(new_theta))) <= OBSTHRESHOLD) return false;
  		}
    }

		return true;
	}

	void getSuccessors(LINK & n, vector<LINK> * s, vector<double>* c) // *** This must be defined
	{
    // cout << "curent g value: " << n.G << endl;
    if (n.G > 16.0) return;
    GRAVITYDIR;
    // if this is a dead end
    if (!n.continue_expansion){
      //cout << "No successor because it is told not to expand" << endl;
      return;
    }
    LINK tn;
    bool display;
    if (searchMode == 0){
      // force balance mode
      //cout <<"Closest point of n: " << n.iClosestP_cost << endl;

      //A-star
      for(double i = 0.0; i <= 1; i+=EL){//expanding length: 0, 0.25, 0.5, 0.75, 1.0
        for (int j = -2; j <= 2; j++){
          display = false;
          //cout << "getting successor in the 1st search" << endl;
          //new position
          tn.x = n.x + (AA * 2 + n.expansion * L) * cos(n.theta);
          tn.y = n.y + (AA * 2 + n.expansion * L) * sin(n.theta);
          //new orientation
          tn.theta = n.theta + (double)j * 22.5 / 180 * PI;

          if (fabs(tn.theta - startLink.theta) > (1.5 * PI)) continue;
          //printf("new link's theta = %g, degree = %g\n", tn.theta, tn.theta / PI * 180);
          tn.expansion = i;
          //update forces status
          tn.forces = n.forces;
          forceDetector(tn);
          if (!isLinkAccessible(tn, n)){
            continue;
          }

          getClosestPt(tn);
          //if (tn.y > 155.0 || tn.x > 130.0) continue;

          // get the closest point to the new link

          tn.getGraAcc(LINKMASS, n.gAccX, n.gAccY, n.gAccT);
          double torq = torqueCheck(tn);

          if (torq < 0.0) {
            //if (tn.theta > PI) printf("An upwards segment neglected due to torque, theta = %g\n", tn.theta);
            over_torque_links.push_back(tn);
            line(realtime_display, cv_plot_coord(tn.x, tn.y),
              cv_plot_coord(tn.getHeadX(), tn.getHeadY()), CV_RGB(255, 0, 255), LINE_THICKNESS);
            continue;
          }

          // calculate cost (distance between this point and the last point of sk_cost)
          double distance;
          if (tn.iClosestP_cost != n.iClosestP_cost){ // if they're not the same point
            if (tn.iClosestP_cost != -1){
              distance = fabs(sk_cost[tn.iClosestP_cost].distanceToEnd - sk_cost[n.iClosestP_cost].distanceToEnd);
            }else{
              distance = 10000.0;
            }
          }else if (tn.iClosestP_cost == (sk_cost.size() - 1)){
            distance = (double)RAND_MAX;
          }else{
            distance = 0.001;
          }

          //draw the new successor in grey
          line(realtime_display, cv_plot_coord(tn.x, tn.y),
            cv_plot_coord(tn.getHeadX(), tn.getHeadY()), CV_RGB(200, 200, 200), LINE_THICKNESS);

          //cout << "got cost value: " << distance / 2 << "(x2)" << endl;
          s->push_back(tn);
          c->push_back(distance / (AA * 2 + L)); // projection on the sk_cost
          display = true;

          n_successors++;
          // cout << "Links expanded: " << n_successors << endl;
        }
      }
    }else if (searchMode == 1){
      // full a star without projection
      for(double i = 0.0; i <= 1; i+=(EL/2)){//expanding length: 0, 0.25, 0.5, 0.75, 1.0
        for (int j = -2; j <= 2; j++){
          //cout << "getting successor in the 2nd search with full AA star" << endl;

          //new position
          tn.x = n.x + (AA * 2 + n.expansion * L) * cos(n.theta);
          tn.y = n.y + (AA * 2 + n.expansion * L) * sin(n.theta);
          //new orientation
          tn.theta = n.theta + (double)j * 22.5 / 180 * PI;
          //printf("new link's theta = %g, degree = %g\n", tn.theta, tn.theta / PI * 180);
          tn.expansion = i;
          //update forces status
          tn.forces = n.forces;
          // getClosestPt(tn);

          if (!isLinkAccessible(tn, n) ){
            //cout << "continued because of inaccessibility" << endl;
            if (!tn.reachGoal(goalLink)) continue;
          }
          getClosestPt(tn);

          tn.getGraAcc(LINKMASS, n.gAccX, n.gAccY, n.gAccT);
          //draw the new successor in grey
          line(realtime_display, cv_plot_coord(tn.x, tn.y),
            cv_plot_coord((tn.x + (tn.expansion * L + AA * 2) * cos(tn.theta)), (tn.y + (tn.expansion * L + AA * 2) * sin(tn.theta))), CV_RGB(200, 200, 200), LINE_THICKNESS);

          s->push_back(tn);
          c->push_back((AA * 2 + L * tn.expansion) / (AA * 2 + L)); // projection on the sk_cost

          display = true;
          n_successors++;
          // cout << "Links expanded: " << n_successors << endl;
        }
      }
    }

    if (n_successors > (SAVE_FLAG * 10000)){
      vector<LINK *> p = reconstructPointerPath(n);// Reconstruct the path with the previous link
      vector<LINK> pathLks;
      for (int i = p.size() - 1; i >= 0; i--)
        pathLks.push_back(* p[i]);
      if (display) pathLks.push_back(tn); // the new successor

      // draw over-torque links
      for(auto i : over_torque_links)
        line(realtime_display, cv_plot_coord(i.x, i.y),
          cv_plot_coord(i.getHeadX(), i.getHeadY()), CV_RGB(255, 0, 255), LINE_THICKNESS);

      // Draw the path
      for(auto i : pathLks)
        line(realtime_display, cv_plot_coord(i.x, i.y),
          cv_plot_coord(i.getHeadX(), i.getHeadY()), CV_RGB(0, 255, 0), LINE_THICKNESS);

      // plot the skeleton
      for(auto i : sk_cost){
        cvPlotPoint(realtime_display, cv_plot_coord(i.x, i.y), CV_RGB(81, 0, 165), VERTEX_SIZE * 2);
      }

      if (searchMode == 1){
        LINK thisLk, lastLk;
        //plot the shortcuts
        for(int b = 0; b < (allLks.size() - 1); b++){
          lastLk = allLks[b];
          thisLk = allLks[b + 1];

          line(realtime_display, cv_plot_coord(lastLk.x, lastLk.y),
              cv_plot_coord(thisLk.getHeadX(), thisLk.getHeadY()), CV_RGB(255, 153, 51), LINE_THICKNESS);
        }

        //plot the robot
        for(auto b : allLks){
          b.plotLINK(realtime_display);
        }
      }

      // plot start link and goal point
      cvPlotPoint(realtime_display, cv_plot_coord(startLink.x, startLink.y), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);
      cvPlotPoint(realtime_display, cv_plot_coord(goalLink.x, goalLink.y), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);

      // namedWindow("Display window", WINDOW_AUTOSIZE);
      sprintf(out_Name, "outfiles/link_path_%d_at_%d_successors.png", searchMode + 1, SAVE_FLAG);
			// imwrite(out_Name, realtime_display);
      imshow("Display window", realtime_display);
      cvWaitKey(1);
      SAVE_FLAG++;

      // Erase the path
      for(auto i : pathLks)
        line(realtime_display, cv_plot_coord(i.x, i.y),
          cv_plot_coord(i.getHeadX(), i.getHeadY()), CV_RGB(200, 200, 200), LINE_THICKNESS);
    }

    //print time and save screenshots
		duration = (clock() - start_time) / (double) CLOCKS_PER_SEC;
		mins = (int)(duration / 60);
		//cout << "Running time: " << mins << " mins " << (duration - 60 * mins) << " secs" << endl;
    /*
    if (flagmin != mins){
      vector<LINK *> px = reconstructPointerPath(n);
      for(auto it : px){
        line(realtime_display, cv_plot_coord((*it).x, (*it).y),
          cv_plot_coord((*it).getHeadX(), (*it).getHeadY()), CV_RGB(0, 200, 0), LINE_THICKNESS);
      }
			sprintf(out_Name, "outfiles/link_path_%d_at_%d_min.png", searchMode + 1, mins);
			imwrite(out_Name, realtime_display);
      // erase
      for(auto it : px){
        line(realtime_display, cv_plot_coord((*it).x, (*it).y),
          cv_plot_coord((*it).getHeadX(), (*it).getHeadY()), CV_RGB(200, 200, 200), LINE_THICKNESS);
      }
			flagmin = mins;
		}
    */
  }

	double getHeuristics(LINK & n)
	{
    if (searchMode == 0){
      // from the closest point on shortest path to the end of that path

      // get the closest point in the sk_heu
      int closestIndex = n.iClosestP_heu;

      //if (closestIndex == -1 || C4 == 0.0) return 0; // if there's no closest point, it goes Djistra

      if (closestIndex < (sk_heu.size() - 1)){ // if the cloeset is not the last point of the path
        // calculate the length from this point to the end

        double acc_length = n.iClosestP_heu == -1? 10000.0 : sk_heu[n.iClosestP_heu].distanceToEnd;
        //cout << "got heuristic value: " << acc_length / 2 << "(x2) and searchMode is " << searchMode << endl;
        return (C4 * acc_length / (AA * 2 + L));
      }else{
        return 0;
      }
    }else{
  		double dx = goalLink.x - n.getHeadX();
  		double dy = goalLink.y - n.getHeadY();
      double h = sqrt(dx * dx + dy * dy);

  		return (h / (AA * 2 + L));
  	}
	}

	vector<LINK> getStartNodes(void)
	{
		vector<LINK> theStartLinks;

    if (searchMode == 0){
      getClosestPt(startLink);
      startLink.getGraAcc(HEADMASS, 0, 0, 0);
      // cout <<"Closest point of startlink: " << startLink.iClosestP_cost << endl;
      theStartLinks.push_back(startLink);

      for(double i = EL; i < 1; i+= EL){
        LINK d_startLink = LINK(startLink.x, startLink.y, startLink.theta, i);
        if (!isLinkAccessible(d_startLink, startLink)) continue;
        getClosestPt(d_startLink);
        d_startLink.getGraAcc(HEADMASS, 0, 0, 0);
        // cout <<"Closest point of d_startLink: " << d_startLink.iClosestP_cost << endl;
        line(realtime_display, cv_plot_coord(d_startLink.x, d_startLink.y),
          cv_plot_coord((d_startLink.x + (d_startLink.expansion * L + AA * 2) * cos(d_startLink.theta)), (d_startLink.y + (d_startLink.expansion * L + AA * 2) * sin(d_startLink.theta))), CV_RGB(200, 200, 200), LINE_THICKNESS);
        theStartLinks.push_back(d_startLink);
      }

    }else{
      //theStartLinks.push_back(startLink);
      getClosestPt(startLink);
      for(double i = 0.0; i <= 1; i+=(EL/2)){//expanding length: 0, 0.25, 0.5, 0.75, 1.0
        for (int j = -2; j <= 2; j++){
          LINK tn;
          //new position
          tn.x = startLink.x + (AA * 2 + startLink.expansion * L) * cos(startLink.theta);
          tn.y = startLink.y + (AA * 2 + startLink.expansion * L) * sin(startLink.theta);
          //new orientation
          tn.theta = startLink.theta + (double)j * 22.5 / 180 * PI;
          tn.expansion = i;

          //update forces status
          tn.forces = startLink.forces;
          tn.getGraAcc(LINKMASS, startLink.gAccX, startLink.gAccY, startLink.gAccT);

          getClosestPt(tn);
          if (!isLinkAccessible(tn, startLink)){
            cout << "Getting startLINK continued because of inaccessibility" << endl;
            continue;
          }
          //draw the new successor in red
          line(realtime_display, cv_plot_coord(tn.x, tn.y),
            cv_plot_coord((tn.x + (tn.expansion * L + AA * 2) * cos(tn.theta)), (tn.y + (tn.expansion * L + AA * 2) * sin(tn.theta))), CV_RGB(255, 0, 0), LINE_THICKNESS);
          theStartLinks.push_back(tn);
        }
      }
    }
		return (theStartLinks);
	}

	bool stopSearch (LINK & n) {
    if (searchMode == 0){ // Check if the path is balanced, no need to reach the goal
      if (n.brace){
        // reject segments pointing back
        if (getClosestIndex(n.getPoint()) > getClosestIndex(n.getHead())) return false;
        // additional conditions
        // if (n.forces.size() < 6) return false;
        // if (n.x < 107.0) return false;
        // if (!((fabs(sin(n.theta) - sin(0.75 * PI)) < INFINITESIMAL_DOUBLE)
        //     && (fabs(cos(n.theta) - cos(0.75 * PI)) < INFINITESIMAL_DOUBLE))) return false;
        if (!(fabs(cos(n.theta) - (-1)) < INFINITESIMAL_DOUBLE)) return false;


        if (forceSpanningCheck(n, false)){
          if(forceSpanningCheck(n, true)) {
            // Reconstruct the path with the previous link
            vector<LINK*> p = reconstructPointerPath(n);
            vector<LINK> pathLks;
            for (int i = p.size() - 1; i >= 0; i--)
              pathLks.push_back(* p[i]);

            if (ChkPthSelf_tangle(pathLks)) return false;

            printf("A force-balanced path found, time: %d mins %g secs\n", mins, duration - 60 * mins);

            Mat decision_display = realtime_display.clone();
            // Draw the path
            for(auto i : pathLks)
              line(decision_display, cv_plot_coord(i.x, i.y),
                  cv_plot_coord((i.x + (i.expansion * L + AA * 2) * cos(i.theta)),
                                (i.y + (i.expansion * L + AA * 2) * sin(i.theta))),
                  CV_RGB(0, 200, 0), LINE_THICKNESS * 2);

            // namedWindow("Display window", WINDOW_AUTOSIZE);
            imshow("Display window", decision_display);
            imwrite("outfiles/decision.png", decision_display);
            cvWaitKey(1);
            printf("Got a valid path. n.theta = %g PI, sin(n.theta) = %g\nDo you want to start the 2nd searching with this output? (y or n) ", n.theta / PI, sin(n.theta));
            string inp;
            cin >> inp;

            if (inp == "y"){
        			paths.push_back(n);
        			return true;
            }else{
              n.continue_expansion = false;
              return false;
            }
          }else{
            printf("tapping satisfied but drilling not satisfied!\n");
            // Mat dia_mat = realtime_display.clone();
            // vector<LINK*> p = reconstructPointerPath(n);
            // vector<LINK> pathLks;
            // for (int i = p.size() - 1; i >= 0; i--)
            //   pathLks.push_back(* p[i]);

            // // Draw the path
            // for(auto i : pathLks)
            //   line(dia_mat, cv_plot_coord(i.x, i.y),
            //     cv_plot_coord((i.x + (i.expansion * L + AA * 2) * cos(i.theta)), (i.y + (i.expansion * L + AA * 2) * sin(i.theta))), CV_RGB(0, 200, 0), LINE_THICKNESS);

            // circle(dia_mat, cv_plot_coord(n.forces.back()[2], n.forces.back()[3]),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(255, 0, 0), -1, CV_AA);

            // //namedWindow( "Diagnose window", WINDOW_AUTOSIZE);
            // imshow("Diagnose window", dia_mat);
            // imwrite("outfiles/diagnose.png", dia_mat);
            // if (n.forces.size() < 6)
            //   cvWaitKey(1);
            return false;
          }
        }else{
          return false;
        }
      }
    }else if (n.reachGoal(goalLink)){
      printf("A path found, time: %d mins %g secs\n", mins, duration - 60 * mins);
  		paths.push_back(n);
			return true;
		}
		return false;
	}
};

#endif