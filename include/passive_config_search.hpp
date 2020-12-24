#ifndef PASSIVECONFIGSEARCH
#define PASSIVECONFIGSEARCH

extern Point ** SkeletonMat;
extern vector<POINT> sk_heu;

class passive_search_LINK : public AStar::Algorithm<passive_search_LINK, LINK, double>{
//class searchLINK : public AStarProblem<LINK>{
public:
  Mat color_map;
	Mat grey_map;
  // Mat realtime_display;
  int SAVE_FLAG;
  vector<LINK> previousLks;

	// variables decsribing problem
	int MAX_X, MIN_X, MAX_Y, MIN_Y;
  LINK startLink, goalLink, drillLink; // drill link pointing at the drill point, opposite from the start link

	// solution classes
	vector<LINK> paths;

	//constructor
	passive_search_LINK(const LINK & startL, const LINK & goalL, const Mat & input_map){
		// read data for planning
    MIN_X = 0;
		MAX_X = original_map.cols;
		MIN_Y = 0;
		MAX_Y = original_map.rows;
    SAVE_FLAG = 0;

    this->startLink = startL;
    this->goalLink = goalL;

    grey_map = input_map.clone();
    // cvtColor(grey_map, realtime_display, CV_GRAY2RGB);
    // resize(realtime_display, realtime_display, Size(), PLOT_SCALE, PLOT_SCALE, INTER_NEAREST);

	}

  bool AreLinksIntersecting(const LINK & link1, const LINK & link2) const {
    double v1 = (link2.getHeadX() - link1.x) * (link1.getHeadY() - link2.getHeadY())
              - (link2.getHeadY() - link1.y) * (link1.getHeadX() - link2.getHeadX());
    double v2 = (link2.x - link1.x) * (link1.getHeadY() - link2.y)
              - (link2.y - link1.y) * (link1.getHeadX() - link2.x);

    if ((v1 * v2) <= 0.0) {
      return true;
    }else{
      return false;
    }
  }

  bool IsPathSelfTangled(const vector<LINK> & inputPath) {
    //check if it crosses itself
    for(int a = 0; a < (inputPath.size() - 2); a++){
      for(int b = a + 2; b < inputPath.size(); b++){
        if (AreLinksIntersecting(inputPath[a], inputPath[b])){
          return true;
        }
      }
    }
    return false;
  }

	bool isLinkAccessible (const LINK & tn, const LINK & ol) {//ol = original LINK
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

  void getClosestPt(LINK & n){ // get the closest point of sk_cost/sk_deu to the end of this link
    int rdY = min(MAX_Y - 1, max(MIN_Y, (int)round(n.getHeadY())));
    int rdX = min(MAX_X - 1, max(MIN_X, (int)round(n.getHeadX())));
    n.iClosestP_cost = SkeletonMat[rdY][rdX].x;
    n.iClosestP_heu = SkeletonMat[rdY][rdX].y;
  }

	void getSuccessors(LINK & n, vector<LINK> * s, vector<double>* c) // *** This must be defined
	{
    LINK tn;

    // full a star without projection
    for(double i = 0.0; i <= 1; i += EL){//expanding length: 0, 0.25, 0.5, 0.75, 1.0
      for (int j = -2; j <= 2; j++){
        //new position
        tn.x = n.getHeadX();
        tn.y = n.getHeadY();
        //new orientation
        tn.theta = n.theta + (double)j * 22.5 / 180 * PI;
        tn.expansion = i;

        // check intersections
        if (!isLinkAccessible(tn, n) ){
          if (!tn.reachGoal(goalLink, PI)) continue;
        }

        getClosestPt(tn);

        s->push_back(tn);
        c->push_back(tn.getLength() / (AA * 2 + L)); // projection on the sk_cost
      }
    }
  }

	double getHeuristics(LINK & n)
	{
    double d = n.getHead().getDistance(sk_heu[n.iClosestP_heu]);

    return 2.0 * (d + sk_heu[n.iClosestP_heu].distanceToEnd) / (AA * 2 + L);
	}

	vector<LINK> getStartNodes(void)
	{
		vector<LINK> theStartLinks;
    LINK tn;
    for(double i = 0.0; i <= 1; i += EL){//expanding length: 0, 0.25, 0.5, 0.75, 1.0
      for (int j = -2; j <= 2; j++){
        //new position
        tn.x = startLink.getHeadX();
        tn.y = startLink.getHeadY();
        //new orientation
        tn.theta = startLink.theta + (double)j * 22.5 / 180 * PI;
        tn.expansion = i;

        if (!isLinkAccessible(tn, startLink)){
          continue;
        }

        getClosestPt(tn);
        theStartLinks.push_back(tn);
      }
    }

		return (theStartLinks);
	}

	bool stopSearch (LINK & n) {
    if (n.reachGoal(goalLink, PI)){
  		paths.push_back(n);
			return true;
		}
		return false;
	}
};

#endif
