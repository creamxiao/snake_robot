#ifndef CONFIGSEARCH
#define CONFIGSEARCH

// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/opencv.hpp"

// #include <dosl/dosl>
// #include "local-include/RSJparser.tcc"
// #include "parameters.hpp"
// #include "LINK.hpp"
// #include "POINT.hpp"
#include <qpOASES.hpp> // quadratic programming
#include "passive_config_search.hpp"

// using namespace std;
// using namespace cv;
USING_NAMESPACE_QPOASES

extern vector<POINT> sk_cost;
extern vector<POINT> sk_heu;
extern vector<LINK> allLks;
extern Point ** SkeletonMat;
extern double MU;
extern double ext_dist;
extern Mat original_map;
extern string program_Name;

class searchLINK : public AStar::Algorithm<searchLINK, LINK, double>{
//class searchLINK : public AStarProblem<LINK>{
public:
	// File names and JSON objects
	string map_image_fName, expt_fName, expt_folderName, expt_Name, out_folderName;
	char out_Name[1024];
	RSJresource expt_container;
  Mat color_map;
	Mat grey_map;

	// Image display variables / parameters
	Mat realtime_display;
	// clock_t start_time;
	double preCalDuration, searchDuration;
  struct timespec start_time, finish_time;
	int SAVE_FLAG;

	// variables decsribing problem
	int MAX_X, MIN_X, MAX_Y, MIN_Y;
  LINK startLink, goalLink, drillLink; // drill link pointing at the drill point, opposite from the start link
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
	searchLINK(LINK startL, LINK goalL, int modeInput){

    SAVE_FLAG = 0;

		// read data for planning
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
    // if (searchMode == 0){
    //   threshold(grey_map, grey_map, 80, 255, 0);
    // }else{
      threshold(grey_map, grey_map, 20, 255, 0);
    // }
    cvtColor(grey_map, color_map, CV_GRAY2RGB);
    resize(color_map, realtime_display, Size(), PLOT_SCALE, PLOT_SCALE, INTER_NEAREST);

    if (searchMode == 0){
      printf("Pre-calculating for heuristic function...");

      clock_gettime(CLOCK_MONOTONIC, &start_time);

      Mat pre_result = realtime_display.clone();

      SkeletonMat = new Point * [MAX_Y];
      for(int j = 0; j < MAX_Y; j++){
        SkeletonMat[j] = new Point [MAX_X];
        for(int i = 0; i < MAX_X; i++){
          if (original_map.at<Vec3b>(j, i).val[0] <= OBSTHRESHOLD){
            SkeletonMat[j][i] = Point(-1, -1);
          }else{
            SkeletonMat[j][i] = PresetClosestPt(Point(i,j), color_map);
            if (SkeletonMat[j][i] != Point(-1,-1)){
              line(pre_result, cv_plot_coord(i, j),
                cv_plot_coord(sk_cost[SkeletonMat[j][i].x]), CV_RGB(200, 200, 200), LINE_THICKNESS);
            }else{
              SkeletonMat[j][i] = Point(0, 0);
            }
          }
        }
      }

      clock_gettime(CLOCK_MONOTONIC, &finish_time);
      preCalDuration = (finish_time.tv_sec - start_time.tv_sec);
      preCalDuration += (finish_time.tv_nsec - start_time.tv_nsec) / 1000000000.0;

      // plot the skeleton
      for(auto i : sk_cost){
        cvPlotPoint(pre_result, cv_plot_coord(i), CV_RGB(0, 165, 81), VERTEX_SIZE * 2);
      }
      LINK drillhead = startL;
      drillhead.reverseLINK();
      drillhead.head = true;
      drillhead.plotLINK(pre_result);
      // plot start and goal
      cvPlotPoint(pre_result, cv_plot_coord(sk_cost.front()), CV_RGB(255, 0, 0), VERTEX_SIZE * 3);
      cvPlotPoint(pre_result, cv_plot_coord(sk_cost.back()), CV_RGB(255, 0, 0), VERTEX_SIZE * 3);

      time_t now = time(0);
	    tm* localtm = localtime(&now);
      imwrite("outfiles/" + program_Name + "_"
              + to_string(1900 + localtm->tm_year) + "_"
              + to_string(1 + localtm->tm_mon) + "_"
              + to_string(localtm->tm_mday) + "_"
              + to_string(localtm->tm_hour) + "_"
              + to_string(localtm->tm_min) + "_"
              + to_string(localtm->tm_sec) + "_shortest_search.png", pre_result);
      cout << "Completed! Pre-cal time: " << preCalDuration << endl;
      cout << GREEN << "Click on 'Display window' then press any key to start searching for the bracing configuration" << RESET << endl;
      putText(pre_result,
        "Pre-calculation completed",
        CvPoint(10, pre_result.rows - 10 - 15),
        FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0, 0, 0), 1, CV_AA, false);
      putText(pre_result,
        "Press any key to continue",
        CvPoint(10, pre_result.rows - 10),
        FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0, 0, 0), 1, CV_AA, false);
      imshow("Display window", pre_result);
      cvWaitKey();
      cout << "Started..." << endl;
      // destroyWindow("Precalculation");
    }

    //realtime_display = color_map.clone();
    //threshold(realtime_display, realtime_display, 200, 255, 0);
		//resize(realtime_display, realtime_display, Size(), PLOT_SCALE, PLOT_SCALE, INTER_NEAREST);

		// plot start link and goal point
		cvPlotPoint(realtime_display, cv_plot_coord(startLink), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);
		cvPlotPoint(realtime_display, cv_plot_coord(goalLink), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);

		//namedWindow( "Display window", WINDOW_AUTOSIZE);
		//imshow("Display window", realtime_display);
		//waitKey();
    line(realtime_display,
        cv_plot_coord(startLink),
			  cv_plot_coord(startLink.getHead()),
        CV_RGB(200, 200, 200), LINE_THICKNESS);

		clock_gettime(CLOCK_MONOTONIC, &start_time);
	}

  bool AreLinksIntersecting(const LINK & link1, const LINK & link2) const {
    double v1 = (link2.getHeadX() - link1.x) * (link1.getHeadY() - link2.getHeadY())
              - (link2.getHeadY() - link1.y) * (link1.getHeadX() - link2.getHeadX());
    double v2 = (link2.x - link1.x) * (link1.getHeadY() - link2.y)
              - (link2.y - link1.y) * (link1.getHeadX() - link2.x);

    if (fabs(v1) + fabs(v2) > 0.0 // v1 and v2 not equal to 0 simultaneously
      && (v1 * v2) <= 0.0) { // v1 * v2 <= 0
      return true;
    }else{
      return false;
    }
  }

  bool IsPathSelfTangled(const vector<LINK> & inputPath) {
    //check if it crosses itself
    for(int a = 0; a < (inputPath.size() - 2); a++){
      for(int b = a + 2; b < inputPath.size(); b++){
        if (AreLinksIntersecting(inputPath[a], inputPath[b])
         && AreLinksIntersecting(inputPath[b], inputPath[a])){
          return true;
        }
      }
    }

    return false;
  }

  void getClosestPt(LINK & n){ // get the closest point of sk_cost/sk_deu to the end of this link
    int rdY = min(MAX_Y - 1, max(MIN_Y, (int)round(n.getHeadY())));
    int rdX = min(MAX_X - 1, max(MIN_X, (int)round(n.getHeadX())));

    Point ptIndex = SkeletonMat[rdY][rdX];
    n.iClosestP_cost = ptIndex.x;
    n.iClosestP_heu = ptIndex.y;
  }

  void forceDetector(LINK & n){//check reaction forces
    //Fx, Fy, x, y, scalar, weight (for combining multiple consecutive forces in the same direction)
    double range = 1.5;
    double first_left = -1, first_right = -1, last_left = -1, last_right = -1;
    //check if any pixel beside the link is obsessed by wall
    for(double i = 3; i < (n.expansion * L + AA * 2 - 3); i++){
			if (grey_map.at<uchar>((int)round(n.y + i * sin(n.theta) + range * sin(n.theta - 0.5 * PI)),
                            (int)round(n.x + i * cos(n.theta) + range * cos(n.theta - 0.5 * PI))) <= OBSTHRESHOLD){
        //left side
        if (first_left == -1) first_left = i;//record the first touch
        if (last_left < i) last_left = i;
      }
      if (grey_map.at<uchar>((int)round(n.y + i * sin(n.theta) + range * sin(n.theta + 0.5 * PI)),
                            (int)round(n.x + i * cos(n.theta) + range * cos(n.theta + 0.5 * PI))) <= OBSTHRESHOLD){
        //right side
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
        if (fabs(n.forces.rbegin()[1][0] - cos(n.theta + 0.5 * PI)) < INFINITESIMAL_DOUBLE
          && fabs(n.forces.rbegin()[1][1] - sin(n.theta + 0.5 * PI)) < INFINITESIMAL_DOUBLE
          && fabs((new_x - n.forces.rbegin()[1][2]) * sin(n.theta)
                - (new_y - n.forces.rbegin()[1][3]) * cos(n.theta)) < INFINITESIMAL_DOUBLE){
          // if the last normal force has the same direction with the new one
          // and whether the wire between two touching points is parallel with the new link
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
      // change the direction for friction
      f[0] = cos(n.theta - (cos(n.theta) * cos(drillLink.theta) + sin(n.theta) * sin(drillLink.theta) < 0 ? PI : 0.0));
      // change the direction for friction
      f[1] = sin(n.theta - (cos(n.theta) * cos(drillLink.theta) + sin(n.theta) * sin(drillLink.theta) < 0 ? PI : 0.0));
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
        if (fabs(n.forces.rbegin()[1][0] - cos(n.theta - 0.5 * PI)) < INFINITESIMAL_DOUBLE
          && fabs(n.forces.rbegin()[1][1] - sin(n.theta - 0.5 * PI)) < INFINITESIMAL_DOUBLE
          && fabs((new_x - n.forces.rbegin()[1][2]) * sin(n.theta)
                - (new_y - n.forces.rbegin()[1][3]) * cos(n.theta)) < INFINITESIMAL_DOUBLE){
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

  bool forceSpanningCheck(LINK & n, bool D_F, const vector<LINK> * pathptr = NULL){ // D_F == true, DRILLFORCE switch on
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

      int nV = n.forces.size(), nC = n.forces.size() + 3 + (*pathptr).size();
      // nV = number of variables, nC = number of constraints
      // nV: forces
      // nC: friction constraints + 3 equilibirm constraints

      real_t HM[nV * nV]; // quadratic terms in objective
      real_t GM[nV]; // linear terms in objective
      real_t AM[nC * nV]; // constraints
      real_t lb[nV]; // lower bounds of variables
      real_t ub[nV]; // upper bounds of variables
      real_t lbA[nC]; // lower bounds of constraints
      real_t ubA[nC]; // upper bounds of constraints

      // initialize HM and GM
      for(int_t i = 0; i < nV; i++ ){
        for(int_t j = 0; j < nV; j++ ){
          HM[i * nV + j] = 0.0;
        }
        GM[i] = 0.0;
      }

      // objective: sum of square of normal forces
      if (!n.forces.empty()){
        for(int i = 0; i < n.forces.size(); i++){
          if (i % 2 != 0) continue;
          HM[i * nV + i] += 2.0;
        }
      }

      for(int_t j = 0; j < nV; j++){
        for(int_t i = 0; i < nC; i++){
          if (i < 2){ // force equilibrium, in two directions
            AM[i * nV + j] = fabs(n.forces[(int)j][(int)i]) < INFINITESIMAL_DOUBLE? 0.0 : n.forces[(int)j][(int)i];
          }else if (i == 2){ // moment equilibrium, in two directions
            AM[i * nV + j] = ((n.forces[(int)j][2] - n.getHeadX()) * n.forces[(int)j][1]
                              - (n.forces[(int)j][3] - n.getHeadY()) * n.forces[(int)j][0]) * P2M_SCALE;
          }else if (i == ((int_t)(j / 2) + 3)){
            AM[i * nV + j] = ((int)j % 2 == 0)? -MU : 1.0; // 0 <= f_i - /mu * N_i
          }else if (i == ((int)(j / 2) + 3 + (int)(n.forces.size() / 2))){
            AM[i * nV + j] = ((int)j % 2 == 0)? MU : 1.0; // f_i + /mu * N_i <= 0
          }else if (i >= (n.forces.size() + 3)
            && (*pathptr)[i - n.forces.size() - 3].forces.size() > j){
            // moment on each segment
            // moment arm of normal forces and frictions
            AM[i * nV + j] = ((n.forces[j][2] - (*pathptr)[i - n.forces.size() - 3].getHeadX())
                             * n.forces[j][1]
                            - (n.forces[j][3] - (*pathptr)[i - n.forces.size() - 3].getHeadY())
                             * n.forces[j][0]) * P2M_SCALE;
          }else{
            AM[i * nV + j] = 0.0;
          }
        }
      }

      for(int_t i = 0; i < nC; i++){
        if (i == 0){
          // force equilibrium, X direction
          lbA[i] = ubA[i] = -(n.gAccX + cos(drillLink.theta + PI) * DRILLFORCE);
        }else if (i == 1){
          // force equilibrium, Y direction
          lbA[i] = ubA[i] = -(n.gAccY + sin(drillLink.theta + PI) * DRILLFORCE);
        }else if (i == 2){
          // moment equilibrium
          lbA[i] = ubA[i] = -n.gAccT - ((drillLink.x - n.getHeadX()) * sin(drillLink.theta + PI)
                              - (drillLink.y - n.getHeadY()) * cos(drillLink.theta + PI))
                              * DRILLFORCE * P2M_SCALE;
        }else if (i < (n.forces.size() / 2 + 3)){
          // friction limit
          lbA[i] = - 2 * MU * MAXFORCE;
          ubA[i] = 0.0;
        }else if (i < (n.forces.size() + 3)){
          // friction limit
          lbA[i] = 0.0;
          ubA[i] = 2 * MU * MAXFORCE;
        }else{
          const LINK & chosenLink = (*pathptr)[i - n.forces.size() - 3];
          // moment on each segment
          lbA[i] = -chosenLink.gAccT
                    - ((drillLink.x - chosenLink.getHeadX()) * sin(drillLink.theta + PI)
                     - (drillLink.y - chosenLink.getHeadY()) * cos(drillLink.theta + PI))
                      * DRILLFORCE * P2M_SCALE
                    - MAXTORQUE;
          ubA[i] = -chosenLink.gAccT
                    - ((drillLink.x - chosenLink.getHeadX()) * sin(drillLink.theta + PI)
                     - (drillLink.y - chosenLink.getHeadY()) * cos(drillLink.theta + PI))
                      * DRILLFORCE * P2M_SCALE
                    + MAXTORQUE;
        }
      }

      for(int_t i = 0; i < nV; i++ ){
        lb[i] = (i % 2 == 0)? 0.0 : -MU * MAXFORCE;
        ub[i] = ((i % 2 == 0)? 1.0 : MU) * MAXFORCE;
      }

      QProblem exampleCP(nV, nC);

      exampleCP.setPrintLevel(PL_NONE);
      int_t nWSR = 100;

      if (SUCCESSFUL_RETURN == exampleCP.init(HM, GM, AM, lb, ub, lbA, ubA, nWSR)){
        // printf("Able to balance drill force: (%g, %g), gravity: (%g, %g) and moment of gravity: %g\n",
                // DRILLFORCE * cos(drillLink.theta + PI),
                // DRILLFORCE * sin(drillLink.theta + PI),
                // n.gAccX, n.gAccY, n.gAccT);
        real_t xOpt[nV];
        exampleCP.getPrimalSolution(xOpt);
        for(int i = 0; i < nV; i++){
          n.forces[i][4] = xOpt[i];
          // printf("Force %s %d, primal scalar %g\n", ((i % 2 == 0)? "(N)": "(F)"), (int)(i / 2), xOpt[i]);
        }
        return true;
      }else{
        return false;
      }
    }else{// tapping
      int nV = n.forces.size() + 1, nC = n.forces.size() + 3;
      // nV = number of variables, nC = number of constraints
      // nV[0 ~ n.forces.size() -1]: forces, nV[n.forces.size()]: Torque at the end of bracing config

      // objective: the torques acting at the end of the link
      real_t HM[nV * nV]; // quadratic terms in objective
      real_t GM[nV]; // linear terms in objective
      real_t AM[nC * nV]; // constraints
      real_t lb[nV]; // lower bounds of variables
      real_t ub[nV]; // upper bounds of variables
      real_t lbA[nC]; // lower bounds of constraints
      real_t ubA[nC]; // upper bounds of constraints

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
              AM[i * nV + j] = ((n.forces[(int)j][2] - n.getHeadX()) * n.forces[(int)j][1]
                              - (n.forces[(int)j][3] - n.getHeadY()) * n.forces[(int)j][0]) * P2M_SCALE;
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
          lbA[i] = ubA[i] = -n.gAccX;
        }else if (i == 1){
          lbA[i] = ubA[i] = -n.gAccY;
        }else if (i == 2){
          lbA[i] = ubA[i] = -n.gAccT;
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

      if (SUCCESSFUL_RETURN == exampleCP.init(HM, GM, AM, lb, ub, lbA, ubA, nWSR)
          && (sqrt(fabs(exampleCP.getObjVal()) * 2) < MAXTORQUE)){
        // if feasible
        return true;
      }

      return false;
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
            AM[i * nV + j] = ((n.forces[(int)j][2] - n.getHeadX()) * n.forces[(int)j][1]
                            - (n.forces[(int)j][3] - n.getHeadY()) * n.forces[(int)j][0]) * P2M_SCALE;
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
        lbA[i] = -n.gAccT - ((drillLink.x - n.getHeadX()) * sin(drillLink.theta + PI)
                            - (drillLink.y - n.getHeadY()) * cos(drillLink.theta + PI)) * DRILLFORCE * P2M_SCALE;
        ubA[i] = -n.gAccT - ((drillLink.x - n.getHeadX()) * sin(drillLink.theta + PI)
                            - (drillLink.y - n.getHeadY()) * cos(drillLink.theta + PI)) * DRILLFORCE * P2M_SCALE;
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
      double min_feasible_torque = sqrt(fabs(exampleCP.getObjVal()) * 2);
      if(min_feasible_torque <= MAXTORQUE){
        return min_feasible_torque;
      }
    }

    return -1.0;
  }

	bool isLinkAccessible (const LINK & tn, LINK & ol) {//ol = original LINK
    if (grey_map.at<uchar>((int)round(tn.getHeadY()),
                           (int)round(tn.getHeadX())) <= OBSTHRESHOLD) return false;
    // is the link in workspace
    if (tn.getHeadX() < MIN_X || tn.getHeadX() > MAX_X
      || tn.getHeadY() < MIN_Y || tn.getHeadY() > MAX_Y)
      return false;
		//check each point on the link, interval = 1, and the end as well
    for(double i = tn.getLength(); i > 0.0; ){
			if (grey_map.at<uchar>((int)round(tn.y + i * sin(tn.theta)),
                             (int)round(tn.x + i * cos(tn.theta))) <= OBSTHRESHOLD) return false;
      i -= 0.5;
		}

    if (ol != startLink){
  		//check if the passage(shortcut) is accessible
  		double y_d = tn.y + (tn.expansion * L + AA * 2) * sin(tn.theta) - ol.y,
            x_d = tn.x + (tn.expansion * L + AA * 2) * cos(tn.theta) - ol.x;
  		double new_theta = atan2(y_d, x_d);
  		for(double i = 1; i <= sqrt(pow(y_d, 2) + pow(x_d, 2)); i+=1){
  			if (grey_map.at<uchar>((int)round(ol.y + i * sin(new_theta)),
                               (int)round(ol.x + i * cos(new_theta))) <= OBSTHRESHOLD) return false;
  		}
    }

		return true;
	}

	void getSuccessors(LINK & n, vector<LINK> * s, vector<double>* c) // *** This must be defined
	{
    LINK tn;
    bool display;
    if (searchMode == 0){
      // force balance mode

      // successors
      for(double i = 0.0; i <= 1; i += ext_dist){//expanding length: 0, 0.5, 1.0
        for (int j = -2; j <= 2; j++){
          display = false;
          //new position
          tn.x = n.getHeadX();
          tn.y = n.getHeadY();
          //new orientation
          tn.theta = n.theta + (double)j * 22.5 / 180 * PI;

          if (fabs(tn.theta - startLink.theta) > (1.5 * PI)) continue;
          tn.expansion = i;
          if (!isLinkAccessible(tn, n)){
            continue;
          }

          //update forces status
          tn.forces = n.forces;
          forceDetector(tn);

          // get the closest point to the new link

          tn.getGraAcc(LINKMASS, n.gAccX, n.gAccY, n.gAccT);
          double torq = torqueCheck(tn);

          if (torq < 0.0){
            line(realtime_display,
                  cv_plot_coord(tn),
                  cv_plot_coord(tn.getHead()),
                  CV_RGB(255, 0, 255), LINE_THICKNESS);
            continue;
          }

          getClosestPt(tn);

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
          line(realtime_display,
            cv_plot_coord(tn),
            cv_plot_coord(tn.getHead()),
            CV_RGB(200, 200, 200), LINE_THICKNESS);

          s->push_back(tn);
          c->push_back(distance / (AA * 2 + L)); // projection on the sk_cost
          display = true;
        }
      }
    }else if (searchMode == 1){
      // full a star without projection
      for(double i = 0.0; i <= 1; i += EL){//expanding length: 0, 0.25, 0.5, 0.75, 1.0
        for (int j = -2; j <= 2; j++){

          //new position
          tn.x = n.getHeadX();
          tn.y = n.getHeadY();
          //new orientation
          tn.theta = n.theta + (double)j * 22.5 / 180 * PI;
          tn.expansion = i;
          //update forces status
          tn.forces = n.forces;

          if (!isLinkAccessible(tn, n) ){
            if (!tn.reachGoal(goalLink)) continue;
          }
          // getClosestPt(tn);

          tn.getGraAcc(LINKMASS, n.gAccX, n.gAccY, n.gAccT);
          //draw the new successor in grey
          line(realtime_display,
              cv_plot_coord(tn),
              cv_plot_coord(tn.getHead()),
              CV_RGB(200, 200, 200), LINE_THICKNESS);

          s->push_back(tn);
          c->push_back(tn.getLength() / (AA * 2 + L)); // projection on the sk_cost

          display = true;
        }
      }
    }

    if (expand_count > (SAVE_FLAG * 10000)){
      Mat process_display = realtime_display.clone();
      vector<LINK *> p = reconstructPointerPath(n);// Reconstruct the path with the previous link
      vector<LINK> pathLks;
      for (int i = p.size() - 1; i >= 0; i--)
        pathLks.push_back(* p[i]);
      if (display) pathLks.push_back(s->back()); // the new successor

      // Draw the path
      for(auto i : pathLks)
        line(process_display,
          cv_plot_coord(i),
          cv_plot_coord(i.getHead()),
          CV_RGB(0, 255, 0), LINE_THICKNESS);

      // plot the skeleton
      for(auto i : sk_cost){
        cvPlotPoint(process_display, cv_plot_coord(i), CV_RGB(81, 0, 165), VERTEX_SIZE * 2);
      }

      clock_gettime(CLOCK_MONOTONIC, &finish_time);
      searchDuration = (finish_time.tv_sec - start_time.tv_sec);
      searchDuration += (finish_time.tv_nsec - start_time.tv_nsec) / 1000000000.0;
      int h = (int)floor(searchDuration / 3600.0);
      int m = (int)floor((searchDuration - h * 3600) / 60.0);
      int s = (int)floor(searchDuration - h * 3600.0 - m * 60.0);
      putText(process_display,
          "Searching...",
          CvPoint(10, process_display.rows - 10 - 15),
          FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0, 0, 0), 1, CV_AA, false);
      putText(process_display,
          "Search time: "
          + (h != 0? to_string(h) + " h " : "")
          + (m != 0? to_string(m) + " m " : "")
          + to_string(s) + " s",
          CvPoint(10, process_display.rows - 10),
          FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0, 0, 0), 1, CV_AA, false);

      if (searchMode == 1){
        //plot the shortcuts
        for(int b = 0; b < (allLks.size() - 1); b++){
          line(process_display,
              cv_plot_coord(allLks[b]),
              cv_plot_coord(allLks[b + 1].getHead()),
              CV_RGB(255, 153, 51), LINE_THICKNESS);
        }

        //plot the robot
        for(auto b : allLks){
          b.plotLINK(process_display);
        }
      }

      // plot start link and goal point
      cvPlotPoint(process_display, cv_plot_coord(startLink.getPoint()), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);
      cvPlotPoint(process_display, cv_plot_coord(goalLink.getPoint()), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);

      // sprintf(out_Name, "outfiles/link_path_%d_at_%d_successors.png", searchMode + 1, SAVE_FLAG);
			// imwrite(out_Name, process_display);
      imshow("Display window", process_display);
      cvWaitKey(1);
      SAVE_FLAG++;
    }
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
      theStartLinks.push_back(startLink);

      for(double i = ext_dist; i < 1; i += ext_dist){
        LINK d_startLink = LINK(startLink.x, startLink.y, startLink.theta, i);
        if (!isLinkAccessible(d_startLink, startLink)) continue;
        getClosestPt(d_startLink);
        d_startLink.getGraAcc(HEADMASS, 0, 0, 0);
        // cout <<"Closest point of d_startLink: " << d_startLink.iClosestP_cost << endl;
        line(realtime_display, cv_plot_coord(d_startLink.x, d_startLink.y),
          cv_plot_coord((d_startLink.x + (d_startLink.expansion * L + AA * 2) * cos(d_startLink.theta)),
                        (d_startLink.y + (d_startLink.expansion * L + AA * 2) * sin(d_startLink.theta))),
                        CV_RGB(200, 200, 200), LINE_THICKNESS);
        theStartLinks.push_back(d_startLink);
      }

    }else{
      getClosestPt(startLink);
      for(double i = 0.0; i <= 1; i += EL){//expanding length: 0, 0.25, 0.5, 0.75, 1.0
        for (int j = -2; j <= 2; j++){
          LINK tn;
          //new position
          tn.x = startLink.x + (AA * 2 + startLink.expansion * L) * cos(startLink.theta);
          tn.y = startLink.y + (AA * 2 + startLink.expansion * L) * sin(startLink.theta);
          //new orientation
          tn.theta = startLink.theta + (double)j * 22.5 / 180 * PI;
          tn.expansion = i;
          if (!isLinkAccessible(tn, startLink)){
            // cout << "Getting startLINK continued because of inaccessibility" << endl;
            continue;
          }

          //update forces status
          tn.forces = startLink.forces;
          tn.getGraAcc(LINKMASS, startLink.gAccX, startLink.gAccY, startLink.gAccT);

          getClosestPt(tn);
          //draw the new successor in red
          line(realtime_display,
              cv_plot_coord(tn),
              cv_plot_coord(tn.getHead()),
              CV_RGB(255, 0, 0), LINE_THICKNESS);
          theStartLinks.push_back(tn);
        }
      }
    }
		return (theStartLinks);
	}

	bool stopSearch (LINK & n) {
    if (searchMode == 0){ // Check if the path is balanced, no need to reach the goal (entrance)
      if (n.brace){
        // reject segments that have no feasible successor and are thus unable to form a feasible passive configuration
        bool haveFeasibleSuc = false;
        LINK tn;
        for(double i = 0.0; i <= 1; i+=(EL/2)){//expanding length: 0, 0.25, 0.5, 0.75, 1.0
          for (int j = -2; j <= 2; j++){
            //new position
            tn.x = n.x + (AA * 2 + n.expansion * L) * cos(n.theta);
            tn.y = n.y + (AA * 2 + n.expansion * L) * sin(n.theta);
            //new orientation
            tn.theta = n.theta + (double)j * 22.5 / 180 * PI;
            tn.expansion = i;

            if (!isLinkAccessible(tn, n) && !tn.reachGoal(goalLink)){
              // if succssor hits wall and doesn't go through the goal
              continue;
            }
            haveFeasibleSuc = true;
            break;
          }
          if (haveFeasibleSuc) break;
        }
        if (!haveFeasibleSuc) return false;

        if (forceSpanningCheck(n, false)){
          // Reconstruct the path with the previous link
          vector<LINK*> p = reconstructPointerPath(n);
          vector<LINK> pathLks;
          for (int i = p.size() - 1; i >= 0; i--){
            pathLks.push_back(* p[i]);
          }
          if (IsPathSelfTangled(pathLks)) return false;

          if(forceSpanningCheck(n, true, & pathLks)) {
            // fast search from current segment to goal to check self-tangling
            passive_search_LINK search2(n, goalLink, grey_map);
            search2.previousLks = pathLks;
            search2.search();
            vector<LINK*> p2 = search2.reconstructPointerPath(search2.paths.front());
            if (p2.empty()) return false; // if not valid path
            for (int i = p2.size() - 1; i >= 0; i--){
              pathLks.push_back(* p2[i]);
            }
            if (IsPathSelfTangled(pathLks)) return false;

            printf("Bracing configuration found, duration: %d mins %g secs\n",
                  (int)floor(searchDuration / 60.0),
                  searchDuration - 60.0 * (int)floor(searchDuration / 60.0));

            Mat decision_display = realtime_display.clone();
            // Draw the path
            for(int i = 0; i < p.size(); i++){
              line(decision_display,
                cv_plot_coord(pathLks[i]),
                cv_plot_coord(pathLks[i].getHead()),
                CV_RGB(0, 200, 0), LINE_THICKNESS * 2);
            }

            clock_gettime(CLOCK_MONOTONIC, &finish_time);
            searchDuration = (finish_time.tv_sec - start_time.tv_sec);
            searchDuration += (finish_time.tv_nsec - start_time.tv_nsec) / 1000000000.0;
            int h = (int)floor(searchDuration / 3600.0);
            int m = (int)floor((searchDuration - h * 3600) / 60.0);
            int s = (int)floor(searchDuration - h * 3600.0 - m * 60.0);
            putText(decision_display,
              "Bracing config finished. Search time: "
                + (h != 0? to_string(h) + " h " : "")
                + (m != 0? to_string(m) + " m " : "")
                + to_string(s) + " s.",
              CvPoint(10, decision_display.rows - 10 - 15),
              FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0, 0, 0), 1, CV_AA, false);
            putText(decision_display,
                "Press any key to continue",
                CvPoint(10, decision_display.rows - 10),
                FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0, 0, 0), 1, CV_AA, false);

            paths.push_back(n);
            imshow("Display window", decision_display);
            // imwrite("outfiles/decision.png", decision_display);
            cout << GREEN << "Got a valid path. Click on 'Display window' then press any key to continue" << RESET << endl;
            cvWaitKey();

            return true;
          }
        }
      }
    }else if (n.reachGoal(goalLink)){
      clock_gettime(CLOCK_MONOTONIC, &finish_time);
      searchDuration = (finish_time.tv_sec - start_time.tv_sec);
      searchDuration += (finish_time.tv_nsec - start_time.tv_nsec) / 1000000000.0;
      printf("Passive supporting configuration found, duration: %d mins %g secs\n",
            (int)floor(searchDuration / 60.0),
            searchDuration - 60.0 * (int)floor(searchDuration / 60.0));
  		paths.push_back(n);
			return true;
		}
		return false;
	}
};

#endif
