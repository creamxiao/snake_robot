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
//#include <opencv/cv.h>
//#include <opencv/cvaux.h>
//#include <opencv/highgui.h>
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

//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/opencv.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
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

#include "parameters.hpp"
#include "POINT.hpp"
#include "LINK.hpp"
#include "skeleton_search.hpp"
#include "configuration_search.hpp"

using namespace std;
using namespace cv;

// Reference Path, for cost function and heuristic function
vector<POINT> sk_cost;
vector<POINT> sk_heu;
vector<LINK> allLks;
Point ** SkeletonMat;

struct timespec start_time, finish_time;

void skeleton_generator(string expt_f_name, string expt_name, string map_name, POINT s, double s_theta, POINT g){
  vector <POINT> landmarks;
  cout << "reading map '" << map_name << "'" << endl;
  // set the landmarks

  //landmarks.push_back(s);
  landmarks.push_back(POINT(s.x + (2 * AA) * cos(s_theta), s.y + (2 * AA) * sin(s_theta)));

  // fill the points
  printf("landmarks.size() = %d\n", landmarks.size());
  for(int i = 0; i < (landmarks.size() - 1); i++){

    sk_cost.push_back(landmarks[i]);

    double angle = atan2(landmarks[i + 1].y - landmarks[i].y, landmarks[i + 1].x - landmarks[i].x);
    double length = sqrt(pow(landmarks[i + 1].y - landmarks[i].y, 2) + pow(landmarks[i + 1].x - landmarks[i].x, 2));

    for (double j = 2; j < length; j += 2){
      sk_cost.push_back(POINT(landmarks[i].x + j * cos(angle), landmarks[i].y + j * sin(angle)));
    }
  }

  // last search section
  printf("skeleton search from (%g, %g) to (%g, %g)\n", landmarks.back().x, landmarks.back().y, g.x, g.y);
  searchSkeleton shortest_search(expt_f_name, expt_name, landmarks.back(), g);
  shortest_search.search();

  printf("shortest path search: start point: (%g, %g), goal point (%g, %g)\n", landmarks.back().x, landmarks.back().y, g.x, g.y);

  if (shortest_search.shortestGoals.empty()){
    cout << "No valid goal point reached!" << endl;
    return;
  }

  vector<POINT*> reconstructed_path = shortest_search.reconstructPointerPath(shortest_search.shortestGoals.back());

  if (reconstructed_path.empty()){
    cout << "No valid sk_cost constructed!" << endl;
    return;
  }
  // put the searched path into sk_cost
  for (int a = reconstructed_path.size() - 1; a >= 0; --a) {
    sk_cost.push_back(*reconstructed_path[a]);
  }
  if (sk_cost.back() != shortest_search.goalNode) sk_cost.push_back(shortest_search.goalNode);
  sk_heu = sk_cost;

  // preset the distance
  sk_cost.front().distanceToEnd = 0.0;
  for (int i = 1; i < sk_cost.size(); i++){
    sk_cost[i].distanceToEnd = sk_cost[i - 1].distanceToEnd + sqrt(pow(sk_cost[i].x - sk_cost[i - 1].x, 2) + pow(sk_cost[i].y - sk_cost[i - 1].y, 2));
    // printf("cost skeleton value[%d]: %g\tincreament: %g\n",i, sk_cost[i].distanceToEnd, sqrt(pow(sk_cost[i].x - sk_cost[i - 1].x, 2) + pow(sk_cost[i].y - sk_cost[i - 1].y, 2)));
  }

  sk_heu.back().distanceToEnd = 0.0;
  for (int i = sk_heu.size() - 2; i >= 0; i--){
    sk_heu[i].distanceToEnd = sk_heu[i + 1].distanceToEnd + sqrt(pow(sk_heu[i].x - sk_heu[i + 1].x, 2) + pow(sk_heu[i].y - sk_heu[i + 1].y, 2));
    // printf("heuristic skeleton value[%d]: %g\tincreament: %g\n",i, sk_heu[i].distanceToEnd, sqrt(pow(sk_heu[i].x - sk_heu[i + 1].x, 2) + pow(sk_heu[i].y - sk_heu[i + 1].y, 2)));
  }

  // draw the sk_cost and calculate the cost
  POINT thisPt, lastPt = sk_cost.front();
  double heuristicCost = 0.0;
  for (int a = 1; a < sk_cost.size(); a++){
    thisPt = sk_cost[a];
    if (round(PLOT_SCALE * (lastPt.x - shortest_search.MIN_X)) == 226 && round(PLOT_SCALE * (lastPt.y - shortest_search.MIN_Y)) == 379){
      cout << "singularity found at (226, 379) scaled" << endl;
    }
    shortest_search.cvPlotPoint(cv_plot_coord(lastPt), CV_RGB(0, 165, 81), VERTEX_SIZE * 2);
    heuristicCost += sqrt(pow(thisPt.x - lastPt.x, 2) + pow(thisPt.y - lastPt.y, 2));
    lastPt = thisPt;
  }

  lastPt = sk_heu.front();
  for (int a = 1; a < sk_heu.size(); a++){
    thisPt = sk_heu[a];
    // shortest_search.cvPlotPoint(cv_plot_coord(lastPt), CV_RGB(81, 0, 165), VERTEX_SIZE * 2);
    heuristicCost += sqrt(pow(thisPt.x - lastPt.x, 2) + pow(thisPt.y - lastPt.y, 2));
    lastPt = thisPt;
  }
  LINK drillhead(s.x, s.y, s_theta, 0.0);
  drillhead.reverseLINK();
  drillhead.head = true;
  drillhead.plotLINK(shortest_search.realtime_display);

  // start and goal point
  shortest_search.cvPlotPoint(cv_plot_coord(g), CV_RGB(255, 0, 0), VERTEX_SIZE * 3);
  shortest_search.cvPlotPoint(cv_plot_coord(s), CV_RGB(255, 0, 0), VERTEX_SIZE * 3);

  cout << "sk_cost size :" << sk_cost.size() << ", heuristic path cost: " << heuristicCost << endl;

  char fileName[1024];
  time_t now = time(0);
	tm* localtm = localtime(&now);
  sprintf(fileName, "outfiles/%d_%d_%d_%d_%d_shortest_search.png", 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, localtm->tm_hour, localtm->tm_min);

  // imwrite(fileName, shortest_search.realtime_display);
  imshow("Display window", shortest_search.realtime_display);
  cout << "Reference path is shown. Press any key to continue" << endl;
  cvWaitKey();
  // destroyWindow("Reference path");
}

int main(int argc, char *argv[])
{
	string program_fName (argv[0]);
	string program_folderName = program_fName.substr(0, program_fName.find_last_of("/\\")+1);

	string expt_f_name = program_folderName + "exptfiles/experiments.json", expt_name = "21a";

	if (argc == 2) {
		// expt_f_name = argv[1];
		expt_name = argv[1];
	}

  // Read from file
  ifstream my_fstream (expt_f_name);
  RSJresource expt_container = RSJresource (my_fstream)[expt_name];
  LINK myStartLink(expt_container["goal"][0].as<int>(), expt_container["goal"][1].as<int>(), (expt_container["angle_g"].as<double>() + 1) * PI, 0);
  LINK myGoalLink(expt_container["start"][0].as<int>(), expt_container["start"][1].as<int>(), (expt_container["angle_s"].as<double>() + 1) * PI, 0);

  printf("myStartLink = LINK(%g, %g, %g, %g)\n", myStartLink.x, myStartLink.y, myStartLink.theta, myStartLink.expansion);
  printf("myGoalLink = LINK(%g, %g, %g, %g)\n",  myGoalLink.x, myGoalLink.y, myGoalLink.theta, myGoalLink.expansion);

  if (expt_container["map_name"].as<std::string>().empty()){
    cout << "Invalid argument! Exited." << endl;
    return 0;
  }else{
    cout << "Now searching for Figure " << expt_name << endl;
  }

  // set the sk_cost
  skeleton_generator(expt_f_name, expt_name, expt_container["map_name"].as<std::string>(), myStartLink.getPoint(), myStartLink.theta, myGoalLink.getPoint());

  // Configuration search from the drilling point
	searchLINK searchInstance_1(expt_f_name, expt_name, program_folderName + "outfiles/", myStartLink, myGoalLink, 0); // last parameter 0 = force balance mode
  searchInstance_1.drillLink = LINK(expt_container["goal"][0].as<int>(), expt_container["goal"][1].as<int>(), expt_container["angle_g"].as<double>() * PI, 0);
	searchInstance_1.search();

  cout << "Path 1 searching accomplished, " << searchInstance_1.expand_count << " vertices have been expanded!" << endl;
  vector<LINK*> path;
  char imgFname[1024];
  if(!searchInstance_1.paths.empty()){
    path = searchInstance_1.reconstructPointerPath(searchInstance_1.paths.back());
  }else{
    cout << "Search 1 never reached the goal!" << endl;
    sprintf(imgFname, "outfiles/%s_C4_%g_debug_search_1.png", argv[0], C4);
    imwrite(imgFname, searchInstance_1.realtime_display);

    imshow("Display window", searchInstance_1.realtime_display);
	  cout << "Press any key to exit" << endl;
    cvWaitKey();
    return 0;
  }
	cout << "Path 1's size: " << path.size() << endl;

	LINK thisLk, lastLk;
	thisLk = * path.back();
	//vector<LINK> allLks;
	allLks.push_back(thisLk);
	double cost = 0.0;

  //calculate for cost and build path 1
	for (int a = path.size() - 2; a >= 0; --a) {
		lastLk = thisLk;

		thisLk = * path[a];
		allLks.push_back(thisLk);

		cost += sqrt(pow(thisLk.x - lastLk.x, 2) + pow(thisLk.y - lastLk.y, 2));
	}

  // update salars of the forces of each LINK
  cout << "updating force scalars" << endl;
  for(int i = 0; i < (allLks.size() - 1); i++){
    for(int j = 0; j < allLks[i].forces.size(); j++){
      // cout << allLks[i].forces[j][4] << " <== " << allLks.back().forces[j][4] << endl;
      allLks[i].forces[j][4] = allLks.back().forces[j][4];
    }
  }
  for(int i = 0; i < allLks.size(); i++){
    if (i == 0) {
      allLks[i].getQP(HEADMASS, cos(myStartLink.theta) * DRILLFORCE, sin(myStartLink.theta) * DRILLFORCE);
    }else{
      allLks[i].getQP(LINKMASS, allLks[i - 1].QPfx, allLks[i - 1].QPfy, allLks[i - 1].QPt);
    }
    // cout << "Segment " << i << ": " << allLks[i].QPfx << ", " << allLks[i].QPfy << ", " << allLks[i].QPt << endl;
  }
  cout << "scalars updating accomplished" << endl;

	//plot the shortcuts
	for(int b = 0; b < (allLks.size() - 1); b++){
		lastLk = allLks[b];
		thisLk = allLks[b + 1];

		line(searchInstance_1.realtime_display, cv_plot_coord(lastLk.x, lastLk.y),
				cv_plot_coord((thisLk.x + (2 * AA + thisLk.expansion * L) * cos(thisLk.theta)),
                      (thisLk.y + (2 * AA + thisLk.expansion * L) * sin(thisLk.theta))),
        CV_RGB(255, 153, 51), LINE_THICKNESS);
	}


  // plot the skeleton
  for(auto i : sk_cost){
    searchInstance_1.cvPlotPoint(searchInstance_1.realtime_display, cv_plot_coord(i.x, i.y), CV_RGB(81, 0, 165), VERTEX_SIZE * 2);
  }

  //plot the path
	for(auto b : allLks){
    //b.head = false;
		b.plotLINK(searchInstance_1.realtime_display);
	}

	// plot start link and goal point
	searchInstance_1.cvPlotPoint(searchInstance_1.realtime_display, cv_plot_coord(searchInstance_1.startLink.x, searchInstance_1.startLink.y), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);
	searchInstance_1.cvPlotPoint(searchInstance_1.realtime_display, cv_plot_coord(searchInstance_1.goalLink.x, searchInstance_1.goalLink.y), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);

  //plot forces
  if (allLks.back().forces.empty()){
    printf("NO forces detected\n");
  }else{
    printf("%d touching point(s) detected\n", allLks.back().forces.size() / 2);
    for(auto i : allLks.back().forces){
      // cout << "Force:";
      // for(auto j : i)
      //   printf(" [%g]", j);
      // cout << endl;
      line(searchInstance_1.realtime_display,
            cv_plot_coord(i[2], i[3]),
            cv_plot_coord(i[2] + i[0] * i[4] / DRILLFORCE * 4 * AA,
                          i[3] + i[1] * i[4] / DRILLFORCE * 4 * AA),
            CV_RGB(100, 179, 179), LINE_THICKNESS * PLOT_SCALE * 2 / 3);
      //arrow head
      line(searchInstance_1.realtime_display,
            cv_plot_coord(i[2] + i[0] * i[4] / DRILLFORCE * 4 * AA,
                          i[3] + i[1] * i[4] / DRILLFORCE * 4 * AA),
            cv_plot_coord(i[2] + i[0] * (i[4] / DRILLFORCE * 4 * AA - ((i[4] > 0) - (i[4] < 0)) * 2) + i[1],
                          i[3] + i[1] * (i[4] / DRILLFORCE * 4 * AA - ((i[4] > 0) - (i[4] < 0)) * 2) - i[0]),
            CV_RGB(100, 179, 179), LINE_THICKNESS * PLOT_SCALE * 2 / 3);
      line(searchInstance_1.realtime_display,
            cv_plot_coord(i[2] + i[0] * i[4] / DRILLFORCE * 4 * AA,
                          i[3] + i[1] * i[4] / DRILLFORCE * 4 * AA),
            cv_plot_coord(i[2] + i[0] * (i[4] / DRILLFORCE * 4 * AA - ((i[4] > 0) - (i[4] < 0)) * 2) - i[1],
                          i[3] + i[1] * (i[4] / DRILLFORCE * 4 * AA - ((i[4] > 0) - (i[4] < 0)) * 2) + i[0]),
            CV_RGB(100, 179, 179), LINE_THICKNESS * PLOT_SCALE * 2 / 3);
      searchInstance_1.cvPlotPoint(searchInstance_1.realtime_display, cv_plot_coord(i[2], i[3]), CV_RGB(0, 179, 179), PLOT_SCALE * VERTEX_SIZE);
    }
  }

  time_t now = time(0);
  tm* localtm = localtime(&now);
  sprintf(imgFname, "outfiles/%s_%d_%d_%d_%d_%d_C4_%g_1st_search_duration_%d_mins_%d_secs_%s.png", argv[0],
          1900 + localtm->tm_year,
          1 + localtm->tm_mon,
          localtm->tm_mday,
          localtm->tm_hour,
          localtm->tm_min, C4,
          (int)floor(searchInstance_1.searchDuration / 60.0),
          (int)ceil(searchInstance_1.searchDuration) % 60, _OS_FLAG == 1? "Linux" : "Mac");
  double search_1_duration = searchInstance_1.searchDuration;
  imwrite(regex_replace(regex_replace(imgFname, regex("\n"), ""), regex(":"), "_"), searchInstance_1.realtime_display);

  // record force and torque acting on each end of the links

  ofstream forceFile;
  sprintf(imgFname, "outfiles/%s_%d_%d_%d_%d_%d_force analysis_%s.txt", argv[0],
          1900 + localtm->tm_year,
          1 + localtm->tm_mon,
          localtm->tm_mday,
          localtm->tm_hour,
          localtm->tm_min, _OS_FLAG == 1? "Linux" : "Mac");
  forceFile.open(imgFname);

  forceFile << "Pre-calculation costs: "
            << searchInstance_1.preCalDuration<< " sec(s)" << endl;
  forceFile << "Bracing configuration costs: "
            << (int)floor(searchInstance_1.searchDuration / 60.0) << " min(s) "
            << searchInstance_1.searchDuration - 60.0 * (int)floor(searchInstance_1.searchDuration / 60.0) << " sec(s), vertices expanded: "
            << searchInstance_1.expand_count << endl;

  imshow("Display window", searchInstance_1.realtime_display);
  cvWaitKey(1);

  // path 2 searching
  searchLINK searchInstance_2(expt_f_name, expt_name, program_folderName + "outfiles/", allLks.back(), myGoalLink, 1); // last parameter 1 = full AA star without projection
  //searchInstance_2.realtime_display = searchInstance_1.realtime_display;
  cout << "For search 2" << endl;
  printf("myStartLink = LINK(%g, %g, %g, %g)\n", searchInstance_2.startLink.x, searchInstance_2.startLink.y, searchInstance_2.startLink.theta, searchInstance_2.startLink.expansion);
  printf("myGoal = LINK(%g, %g, %g, %g)\n",  searchInstance_2.goalLink.x, searchInstance_2.goalLink.y, searchInstance_2.goalLink.theta, searchInstance_2.goalLink.expansion);

  searchInstance_2.search();
  cout << "Path 2 searching accomplished, " << searchInstance_1.expand_count + searchInstance_2.expand_count << " vertices have been expanded in total!" << endl;
  if(searchInstance_2.paths.empty()){
    cout << "Didn't found valid path 2!\nSearching ended." << endl;
    sprintf(imgFname, "outfiles/%s_C4_%g_debug_search_2.png", argv[0], C4);
    imwrite(imgFname, searchInstance_2.realtime_display);

    imshow("Display window", searchInstance_2.realtime_display);
	  cout << "Press any key to continue" << endl;
    cvWaitKey();
    return 0;
  }

  vector<LINK*> path_2 = searchInstance_2.reconstructPointerPath(searchInstance_2.paths.back());
  cout << "path 2's size: " << path_2.size() << endl;

  // build path 2
  for (int a = path_2.size() - 1; a >= 0; --a) {
    allLks.push_back(* path_2[a]);
  }

  //plot the shortcuts
	for(int b = 0; b < (allLks.size() - 1); b++){
		lastLk = allLks[b];
		thisLk = allLks[b + 1];

		line(searchInstance_2.realtime_display, cv_plot_coord(lastLk.x, lastLk.y),
				cv_plot_coord((thisLk.x + (2 * AA + thisLk.expansion * L) * cos(thisLk.theta)), (thisLk.y + (2 * AA + thisLk.expansion * L) * sin(thisLk.theta))), CV_RGB(255, 153, 51), LINE_THICKNESS);
	}


  // plot the skeleton
  for(auto i : sk_cost){
    searchInstance_2.cvPlotPoint(searchInstance_2.realtime_display, cv_plot_coord(i.x, i.y), CV_RGB(81, 0, 165), VERTEX_SIZE * 2);
  }

  //plot the path
	for(auto b : allLks){
		b.plotLINK(searchInstance_2.realtime_display);
	}

	// plot start link and goal point
	searchInstance_2.cvPlotPoint(searchInstance_2.realtime_display, cv_plot_coord(searchInstance_1.startLink.x, searchInstance_1.startLink.y), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);
	searchInstance_2.cvPlotPoint(searchInstance_2.realtime_display, cv_plot_coord(searchInstance_2.goalLink.x, searchInstance_2.goalLink.y), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);

  //allLks.back().makeHead();
	printf("Path's total length: %g, %d link(s)\n", cost, allLks.size());

  //plot forces
  printf("%d touching point(s) detected\n", allLks.back().forces.size() / 2);
  for(auto i : allLks.back().forces){
    cout << "Force:";
    for(auto j : i)
      printf(" [%g]", j);
    cout << endl;
    line(searchInstance_2.realtime_display, cv_plot_coord(i[2], i[3]),
        cv_plot_coord(i[2] + i[0] * i[4] / DRILLFORCE * 4 * AA,
                      i[3] + i[1] * i[4] / DRILLFORCE * 4 * AA),
        CV_RGB(100, 179, 179), LINE_THICKNESS * PLOT_SCALE * 2 / 3);
    //arrow head
    line(searchInstance_2.realtime_display,
          cv_plot_coord(i[2] + i[0] * i[4] / DRILLFORCE * 4 * AA,
                        i[3] + i[1] * i[4] / DRILLFORCE * 4 * AA),
          cv_plot_coord(i[2] + i[0] * (i[4] / DRILLFORCE * 4 * AA - ((i[4] > 0) - (i[4] < 0)) * 2) + i[1],
                        i[3] + i[1] * (i[4] / DRILLFORCE * 4 * AA - ((i[4] > 0) - (i[4] < 0)) * 2) - i[0]),
          CV_RGB(100, 179, 179), LINE_THICKNESS * PLOT_SCALE * 2 / 3);
    line(searchInstance_2.realtime_display,
          cv_plot_coord(i[2] + i[0] * i[4] / DRILLFORCE * 4 * AA,
                        i[3] + i[1] * i[4] / DRILLFORCE * 4 * AA),
          cv_plot_coord(i[2] + i[0] * (i[4] / DRILLFORCE * 4 * AA - ((i[4] > 0) - (i[4] < 0)) * 2) - i[1],
                        i[3] + i[1] * (i[4] / DRILLFORCE * 4 * AA - ((i[4] > 0) - (i[4] < 0)) * 2) + i[0]),
          CV_RGB(100, 179, 179), LINE_THICKNESS * PLOT_SCALE * 2 / 3);
    searchInstance_2.cvPlotPoint(searchInstance_2.realtime_display, cv_plot_coord(i[2], i[3]),
                                  CV_RGB(0, 179, 179), PLOT_SCALE * VERTEX_SIZE);
  }

	now = time(0);
	localtm = localtime(&now);
	sprintf(imgFname, "outfiles/%s_%d_%d_%d_%d_%d_C4_%g_2nd_search_duration_%d_mins_%d_secs_%s.png", argv[0],
          1900 + localtm->tm_year,
          1 + localtm->tm_mon,
          localtm->tm_mday,
          localtm->tm_hour,
          localtm->tm_min, C4,
          (int)floor((searchInstance_2.searchDuration + searchInstance_1.searchDuration) / 60.0),
          (int)ceil(searchInstance_2.searchDuration + searchInstance_1.searchDuration) % 60, _OS_FLAG == 1? "Linux" : "Mac");
	imwrite(regex_replace(regex_replace(imgFname, regex("\n"), ""), regex(":"), "_"), searchInstance_2.realtime_display);

  forceFile << "Passive supporting configuration costs: "
            << (int)floor(searchInstance_2.searchDuration / 60.0) << " min(s) "
            << searchInstance_2.searchDuration - 60.0 * (int)floor(searchInstance_2.searchDuration / 60.0) << " sec(s), vertices expanded: "
            << searchInstance_2.expand_count << endl;
  forceFile << "Total configuration costs: "
            << (int)floor((search_1_duration + searchInstance_2.searchDuration) / 60.0) << " min(s) "
            << (searchInstance_2.searchDuration + searchInstance_1.searchDuration)
                          - 60.0 * (int)floor((searchInstance_2.searchDuration + searchInstance_1.searchDuration) / 60.0) << " sec(s), vertices expanded: "
            << searchInstance_1.expand_count + searchInstance_2.expand_count << endl;
	imshow("Display window", searchInstance_2.realtime_display);
	cvWaitKey(1);

// combination picture =============================================

  vector<LINK> dis_path = allLks;
  reverse(dis_path.begin(), dis_path.end());
  for (auto & i : dis_path){
    i.reverseLINK();
  }

  dis_path.back().head = true;

  Mat forpaper = searchInstance_2.color_map.clone();
  resize(forpaper, forpaper, Size(), PLOT_SCALE, PLOT_SCALE, INTER_NEAREST);

  //plot the shortcuts
	for(int b = 0; b < (dis_path.size() - 1); b++){
		lastLk = dis_path[b];
		thisLk = dis_path[b + 1];

		line(forpaper, cv_plot_coord(lastLk.x, lastLk.y),
				cv_plot_coord((thisLk.x + (2 * AA + thisLk.expansion * L) * cos(thisLk.theta)), (thisLk.y + (2 * AA + thisLk.expansion * L) * sin(thisLk.theta))), CV_RGB(255, 153, 51), LINE_THICKNESS);
	}

  // plot the skeleton
  for(auto i : sk_cost){
    searchInstance_2.cvPlotPoint(forpaper, cv_plot_coord(i.x, i.y), CV_RGB(81, 0, 165), VERTEX_SIZE * 2);
  }

  //plot the path
	for(auto b : dis_path){
		b.plotLINK(forpaper);
	}

  //plot forces
  for(auto i : dis_path.front().forces){
    line(forpaper, cv_plot_coord(i[2], i[3]),
          cv_plot_coord(i[2] + i[0] * i[4] / DRILLFORCE * 4 * AA,
                        i[3] + i[1] * i[4] / DRILLFORCE * 4 * AA),
          CV_RGB(100, 179, 179), LINE_THICKNESS * PLOT_SCALE * 2 / 3);
    //arrow head
    line(forpaper, cv_plot_coord(i[2] + i[0] * i[4] / DRILLFORCE * 4 * AA,
                                  i[3] + i[1] * i[4] / DRILLFORCE * 4 * AA),
        cv_plot_coord(i[2] + i[0] * (i[4] / DRILLFORCE * 4 * AA - ((i[4] > 0) - (i[4] < 0)) * 2) + i[1],
                      i[3] + i[1] * (i[4] / DRILLFORCE * 4 * AA - ((i[4] > 0) - (i[4] < 0)) * 2) - i[0]),
        CV_RGB(100, 179, 179), LINE_THICKNESS * PLOT_SCALE * 2 / 3);
    line(forpaper, cv_plot_coord(i[2] + i[0] * i[4] / DRILLFORCE * 4 * AA, i[3] + i[1] * i[4] / DRILLFORCE * 4 * AA),	cv_plot_coord(i[2] + i[0] * (i[4] / DRILLFORCE * 4 * AA - ((i[4] > 0) - (i[4] < 0)) * 2) - i[1], i[3] + i[1] * (i[4] / DRILLFORCE * 4 * AA - ((i[4] > 0) - (i[4] < 0)) * 2) + i[0]), CV_RGB(100, 179, 179), LINE_THICKNESS * PLOT_SCALE * 2 / 3);
    searchInstance_2.cvPlotPoint(forpaper, cv_plot_coord(i[2], i[3]), CV_RGB(0, 179, 179), PLOT_SCALE * VERTEX_SIZE);
  }

	// plot start link and goal point
	searchInstance_2.cvPlotPoint(forpaper, cv_plot_coord(searchInstance_1.startLink.x, searchInstance_1.startLink.y), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);
	searchInstance_2.cvPlotPoint(forpaper, cv_plot_coord(searchInstance_2.goalLink.x, searchInstance_2.goalLink.y), CV_RGB(255, 0, 0), PLOT_SCALE * VERTEX_SIZE);

  sprintf(imgFname, "outfiles/%s_%d_%d_%d_%d_%d_forPaper_%s.png", argv[0], 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, localtm->tm_hour, localtm->tm_min, _OS_FLAG == 1? "Linux" : "Mac");
	imwrite(regex_replace(regex_replace(imgFname, regex("\n"), ""), regex(":"), "_"), forpaper);
	imshow("Display window", forpaper);
	cout << "Press any key to save and exit" << endl;
  cvWaitKey();

  // ================================================================================================================


  forceFile << imgFname << endl << endl; // title line
  forceFile << "In World Coordinate System (WCS), positive y goes downwards; in LINK Coordinate System (LCS) for both ends, positive x goes towards the center of the link, positive y goes clockwise lateral; positive torque in both systems goes inwards the screen\nIndex of LINK starting from the head of the robot, data format: (force in X LCS (N), force in Y LCS (N), torque (Nm)) (force in X WCS (N), force in Y WCS (N), torque (Nm))" << endl << endl;

  GRAVITYDIR;
  forceFile << "Gravity: [" << g[0] << ", " << g[1] << "]" << " m/s^2" << endl;
  forceFile << "Drill force: " << DRILLFORCE << " N" << endl;
  forceFile << "Static friction coefficient: " << MU << endl;
  forceFile << "Mass of regular LINK: " << LINKMASS << " kg" << endl;
  forceFile << "Mass of head LINK: " << HEADMASS << " kg" << endl << endl;

  forceFile << "External force summary: " << allLks.back().forces.size() / 2 << " touching point(s) detected" << endl;
  forceFile << "Forces\t\t\tDirection X\tDirection Y\tPosition X\tPosition Y\tNorm(N)\tWeight" << endl;
  double greatestNormalForce = 0.0, greatestFriction = 0.0;
  for(int i = 0; i < allLks.back().forces.size(); i++){
    forceFile << ((i % 2) == 0? "Normal force " : "Friction ") << floor((double)i / 2);
    if ((i % 2) == 0){
      greatestNormalForce = max(greatestNormalForce, fabs(allLks.back().forces[i][4]));
    }else{
      greatestFriction = max(greatestFriction, fabs(allLks.back().forces[i][4]));
    }
    for(auto j : allLks.back().forces[i]){
      forceFile << "\t\t" << j;
    }
    forceFile << endl;
  }
  forceFile << endl;

  double xf = cos(expt_container["angle_g"].as<double>() * PI + PI) * DRILLFORCE,
          yf = sin(expt_container["angle_g"].as<double>() * PI + PI) * DRILLFORCE,
          tt = 0.0;
  int forceNumber = 0;
  double greatestForce = 0.0, greatestTorque = 0.0;
  double forceModule;
  for(int b = 0; b < allLks.size(); b++){

    // bool newforce = false;
    if (allLks[b].brace && forceNumber != allLks[b].forces.size()){ // if any new force found
        // newforce = true;
        forceFile << "Normal force " << floor((double)allLks[b].forces.size() / 2) << " and Friction " << floor((double)allLks[b].forces.size() / 2) << " acting on LINK " << b << endl;
    }

    forceFile << "LINK " << b << ":";
    if (b == 0){
      forceFile << "\t\tfront(" << xf * cos(allLks[b].theta) + yf * sin(allLks[b].theta) << ", " << xf * sin(allLks[b].theta) - yf * cos(allLks[b].theta)  << ", " << tt
                << ") ("<< xf << ", " << yf << ", " << tt << ")" << endl;
    }

    // torque generated by parent force
    tt += ((2 * AA + L * allLks[b].expansion) * cos(allLks[b].theta + PI) * yf
          - (2 * AA + L * allLks[b].expansion) * sin(allLks[b].theta + PI) * xf)
          * P2M_SCALE;
    // torque generated by gravity
    tt += ((AA + L * 0.5 * allLks[b].expansion) * cos(allLks[b].theta + PI) * g[1]
          - (AA + L * 0.5 * allLks[b].expansion) * sin(allLks[b].theta + PI) * g[0])
          * (b == 0? HEADMASS : LINKMASS) * P2M_SCALE;
    // gravity acting on this link
    xf += g[0] * (b == 0? HEADMASS : LINKMASS);
    yf += g[1] * (b == 0? HEADMASS : LINKMASS);
    if (allLks[b].brace && forceNumber != allLks[b].forces.size()){
      forceNumber = allLks[b].forces.size();
      // forces acting on this link
      xf += allLks[b].forces.rbegin()[1][0] * allLks[b].forces.rbegin()[1][4] + allLks[b].forces.rbegin()[0][0] * allLks[b].forces.rbegin()[0][4]; // normal force + friction in x direction
      yf += allLks[b].forces.rbegin()[1][1] * allLks[b].forces.rbegin()[1][4] + allLks[b].forces.rbegin()[0][1] * allLks[b].forces.rbegin()[0][4]; // normal force + friction in y direction
      // torque generated by formal force
      tt += ((allLks[b].forces.rbegin()[1][2] - allLks[b].getHeadX()) * allLks[b].forces.rbegin()[1][1]
             - (allLks[b].forces.rbegin()[1][3] - allLks[b].getHeadY()) * allLks[b].forces.rbegin()[1][0])
              * allLks[b].forces.rbegin()[1][4] * P2M_SCALE;
      // torque generated by friction
      tt += ((allLks[b].forces.rbegin()[0][2] - allLks[b].getHeadX()) * allLks[b].forces.rbegin()[0][1]
             - (allLks[b].forces.rbegin()[0][3] - allLks[b].getHeadY()) * allLks[b].forces.rbegin()[0][0])
              * allLks[b].forces.rbegin()[0][4] * P2M_SCALE;
    }
    forceModule = sqrt(pow(xf, 2) + pow(yf, 2));
    greatestForce = max(greatestForce, fabs(forceModule));
    greatestTorque = max(greatestTorque, fabs(tt));
    forceFile << "\t\tback(" << xf * cos(allLks[b].theta) + yf * sin(allLks[b].theta) << ", " << xf * sin(allLks[b].theta) - yf * cos(allLks[b].theta)  << ", " << -tt << ") ("<< -xf << ", " << -yf << ", " << -tt << ")";
    forceFile << " module(" << forceModule << ", " << fabs(tt) << ")" << endl;

  }
  forceFile << "On segment joints:\n greatest force: " << greatestForce << ", greatest torque: " << greatestTorque << " (all are in absolute value)" << endl;

  forceFile << "Geatest normal force: " << greatestNormalForce << ", greatest friction: " << greatestFriction << " (all are in absolute value)" << endl;

  forceFile << "skeleton size: " << sk_cost.size() << ", distance to end: " << sk_cost.back().distanceToEnd * P2M_SCALE << endl;
  forceFile.close();

  // record force and torque acting on each end of the links
  ofstream configFile;
  sprintf(imgFname, "outfiles/%s_%d_%d_%d_%d_%d_configuration.txt", argv[0], 1900 + localtm->tm_year, 1 + localtm->tm_mon, localtm->tm_mday, localtm->tm_hour, localtm->tm_min);
  configFile.open(imgFname);
  configFile << imgFname << endl << endl; // title line
  configFile << "Data format: x, y, theta, expansion" << endl << endl;

  for(int i = 0; i < allLks.size(); i++){
    configFile << allLks[i].x << ", " << allLks[i].y << ", " << allLks[i].theta << ", " << allLks[i].expansion << endl;
  }
  configFile << endl;

  configFile.close();

  return 0;
}
