#ifndef SKELETONSEARCH
#define SKELETONSEARCH

// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/opencv.hpp"

// #include <dosl/dosl>
// #include "local-include/RSJparser.tcc"
// #include "POINT.hpp"
// using namespace std;
// using namespace cv;

//class searchSkeleton : public DOSL_CLASS(Problem)<POINT,double>
class searchSkeleton : public AStar::Algorithm<searchSkeleton, POINT, double>{
public:
    // Fime names and JSON objects
    std::string map_image_fName, expt_fName, expt_folderName, expt_Name;
    RSJresource expt_container;
    Mat original_map, grey_map;

    // Image display variables / parameters
    Mat realtime_display;

    // variables decsribing problem
    COORD_TYPE MAX_X, MIN_X, MAX_Y, MIN_Y;
    POINT startNode, goalNode;
    // solution classes
  	vector<POINT> shortestGoals;

    // -----------------------------------------------------------

    void cvPlotPoint(CvPoint pt, CvScalar colr, int size) {
      for (int i = (pt.x - (int)((size * 2 - 1) / 2)); i <= (pt.x + (int)ceil((size * 2 - 1) / 2)); i++){
  			for (int j = (pt.y - (int)((size * 2 - 1) / 2)); j <= (pt.y + (int)ceil((size * 2 - 1) / 2)); j++){
  				if (i<0 || i>=realtime_display.cols || j<0 || j>=realtime_display.rows) continue;
          realtime_display.at<Vec3b>(j,i).val[0] = (uchar)colr.val[0];
          realtime_display.at<Vec3b>(j,i).val[1] = (uchar)colr.val[1];
          realtime_display.at<Vec3b>(j,i).val[2] = (uchar)colr.val[2];
        }
      }
    }

    // Constructor
    searchSkeleton (std::string expt_f_name, std::string expt_name, POINT startP, POINT goalP)
    {
        expt_fName = expt_f_name; expt_Name = expt_name;
        expt_folderName = expt_fName.substr(0, expt_fName.find_last_of("/\\")+1);

        // Read from file
        std::ifstream my_fstream (expt_fName);
        expt_container = RSJresource (my_fstream)[expt_Name];
        map_image_fName = expt_folderName + expt_container["map_name"].as<std::string>();
        original_map = imread(map_image_fName, CV_LOAD_IMAGE_COLOR); // second parameter computes representative points

        // read data for planning
        MAX_X = original_map.size().width;
        MIN_X = 0;
        MAX_Y = original_map.size().height;
        MIN_Y = 0;
        startNode = startP;
        goalNode = goalP;

        cvtColor(original_map, grey_map, CV_RGB2GRAY);
        threshold(grey_map, grey_map, 20, 255, 0);
        cvtColor(grey_map, grey_map, CV_GRAY2RGB);
    		resize(grey_map, realtime_display, Size(), PLOT_SCALE, PLOT_SCALE, INTER_NEAREST);
    }

    // -----------------------------------------------------------

    bool isNodeInWorkspace (const POINT & tn) {
      if ( tn.x < MIN_X || tn.x > MAX_X || tn.y < MIN_Y || tn.y > MAX_Y )  return (false);
      return (true);
    }

    bool isPOINTAccessible (const POINT & tn) {
      if (!isNodeInWorkspace(tn))  return (false);
      if (original_map.at<Vec3b>((int)round(tn.y), (int)round(tn.x)).val[0] <= OBSTHRESHOLD) return false; // red

      return (true);
    }

    void getSuccessors (POINT & n, vector<POINT> * s, vector<double> * c) // *** This must be defined
    {
      // This function should account for obstacles and size of environment.
      POINT tn;
      //double d_grid[] = {-1, -sin(PI/8), 0, sin(PI/8), 1};
      for (int a = -2; a <= 2; a+=2){
        for (int b = -2; b <= 2; b+=2) {
          //if (-1 <= a && a <= 1 && -1 <= b && b <= 1) continue;
          if (a == 0 && b == 0) continue;

          tn.x = n.x + (double)a;
          tn.y = n.y + (double)b;

          if (!isPOINTAccessible(tn)){
          	continue;
          }

          s->push_back(tn);
          double dx = tn.x - n.x, dy = tn.y - n.y, op = 1.0;

          if (original_map.at<Vec3b>((int)round(tn.y), (int)round(tn.x)).val[2] < 200){
            op = 0.4;
            // printf("get a cost discount at (%g, %g)\n", round(tn.x), round(tn.y));
          }

          c->push_back(op * sqrt(dx * dx + dy * dy));
        }
      }
    }

    // -----------------------------------------------------------

    double getHeuristics (POINT & n)
    {
        double dx = goalNode.x - n.x;
        double dy = goalNode.y - n.y;
        //return sqrt(dx * dx + dy * dy);
        return (0.0);
    }

    // -----------------------------------------------------------

    vector<POINT> getStartNodes (void)
    {
        vector<POINT> startNodes;

        startNodes.push_back (startNode);

        return (startNodes);
    }

    // -----------------------------------------------------------

    bool stopSearch (POINT & n) {
        if ((pow(n.x - goalNode.x, 2) + pow(n.y - goalNode.y, 2)) < 4) {
          printf("Got close to the goal: (%g, %g)\n", n.x, n.y);
            shortestGoals.push_back(n);
            return (true);
        }
        return (false);
    }
};

#endif