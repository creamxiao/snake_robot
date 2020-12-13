#ifndef LIN
#define LIN

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "POINT.hpp"
using namespace std;
using namespace cv;

class LINK : public AStar::Node<LINK, double>
{
public:
  double x, y;
	double theta; //orientation, radian
	double expansion; // 0 - 1
  bool head = false;
  bool brace;// = false;
  bool continue_expansion = true;
  int iClosestP_cost = -1; // the index of closest point in the sk_cost to the end of this LINK
  int iClosestP_heu = -1;
  vector<vector<double> > forces;//Fx(0), Fy(1), x(2), y(3), scalar(4), weight(5)
  double gAccX, gAccY; // accumulated gravity on this link
  double gAccT; // accumulated gravity-torque about the end of this link

  double QPfx, QPfy; // force on this link from quadratic programing
  double QPt; // torque about the end of this link from quadratic programing

	LINK() {
    brace = false;
    gAccX = 0;
    gAccY = 0;
    gAccT = 0;
    QPfx = 0;
    QPfy = 0;
    QPt = 0;
  }

	LINK(double x, double y, double theta, double expansion)
	{
		this->x = x;
		this->y = y;
		this->theta = theta;
		this->expansion = expansion;
    brace = false;
    gAccX = 0;
    gAccY = 0;
    gAccT = 0;
    QPfx = 0;
    QPfy = 0;
    QPt = 0;
  }

  void reverseLINK(){
    double ox = this->x, oy = this->y, otheta = this->theta;
    this->x = ox + (2 * AA + L * this->expansion) * cos(otheta);
    this->y = oy + (2 * AA + L * this->expansion) * sin(otheta);
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
    for(double i = this->getLength(); i > 0.0; i-=0.4){
      if(sqrt(pow(this->x + i * cos(this->theta) - n.x, 2) + pow(this->y + i * sin(this->theta)- n.y, 2)) <= 1) {
        //printf("head theta: %.3f PI, goal theta: %.3f PI, fabs(this->theta - n.theta) = %g\n", this->theta / PI, n.theta / PI, fabs(this->theta - n.theta));
        return true;
      }
    }
    return false;

		//return (sqrt(pow(x + (expansion * L + AA * 2) * cos(theta) - n.x, 2) + pow(y + (expansion * L + AA * 2) * sin(theta)- n.y, 2)) <= 1);
	}

	POINT getPoint() const{
		return POINT(this->x, this->y);
	}

  POINT getHead() const{
    return POINT(this->x + (2 * AA + L * this->expansion) * cos(this->theta), this->y + (2 * AA + L * this->expansion) * sin(this->theta));
  }

  double getHeadX() const{
    return (this->x + (2 * AA + L * this->expansion) * cos(this->theta));
  }

  double getHeadY() const{
    return (this->y + (2 * AA + L * this->expansion) * sin(this->theta));
  }

  void makeHead(){
		this->head = true;
    this->expansion = 0;

	}

  double getLength() const{
    return (AA * 2 + L * this->expansion);
  }

	void plotLINK(Mat & plot_canvas){
		//first half of link red
		line(plot_canvas, cv_plot_coord(this->x, this->y),
				cv_plot_coord((this->x + AA * cos(this->theta)),
                      (this->y + AA * sin(this->theta))),
        CV_RGB(255, (this->brace? 0 : 77), (this->brace? 0 : 77)), LINE_THICKNESS * PLOT_SCALE, CV_AA);

		if (this->expansion > 0){
			//expansion blue
			line(plot_canvas, cv_plot_coord((this->x + AA * cos(this->theta)), (this->y + AA * sin(this->theta))),
					cv_plot_coord((this->x + (AA + this->expansion * L) * cos(this->theta)),
                        (this->y + (AA + this->expansion * L) * sin(this->theta))),
          CV_RGB((this->brace? 0 : 77), (this->brace? 0 : 77), 255), LINE_THICKNESS * PLOT_SCALE, CV_AA);
		}
		//second half of link green, if head, black
    if (this->head){
      line(plot_canvas,
          cv_plot_coord(this->x + (AA + this->expansion * L) * cos(this->theta),
                        this->y + (AA + this->expansion * L) * sin(this->theta)),
  				cv_plot_coord(this->x + (2 * AA + this->expansion * L) * cos(this->theta),
                        this->y + (2 * AA + this->expansion * L) * sin(this->theta)),
          CV_RGB(0, 0, 0), LINE_THICKNESS * PLOT_SCALE * 2 / 3);
      circle(plot_canvas,
            cv_plot_coord(this->x + (1.5 * AA + this->expansion * L) * cos(this->theta),
                          this->y + + (1.5 * AA + this->expansion * L) * sin(this->theta)),
          PLOT_SCALE * VERTEX_SIZE , CV_RGB(0, 0, 0), -1, CV_AA);
    }
    else{
  		line(plot_canvas,
          cv_plot_coord((this->x + (AA + this->expansion * L) * cos(this->theta)),
                        (this->y + (AA + this->expansion * L) * sin(this->theta))),
  				cv_plot_coord((this->x + (2 * AA + this->expansion * L) * cos(this->theta)),
                        (this->y + (2 * AA + this->expansion * L) * sin(this->theta))),
          CV_RGB((this->brace? 0 : 77), 255, (this->brace? 0 : 77)), LINE_THICKNESS * PLOT_SCALE, CV_AA);
    }
		//origin of link
    circle(plot_canvas, cv_plot_coord(this->x, this->y),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(50, 50, 50), -1, CV_AA);
		//circle(plot_canvas, cv_plot_coord((this->x + (2 * AA + this->expansion * L) * cos(this->theta)), (this->y + (2 * AA + this->expansion * L) * sin(this->theta))),	PLOT_SCALE * VERTEX_SIZE , CV_RGB(50, 50, 50), -1, CV_AA);
  }

  void getGraAcc(double mass, double parentAccFX = 0.0, double parentAccFY = 0.0, double parentAccT = 0.0){ // the torque is about the end of this LINK
    GRAVITYDIR;
    gAccX = parentAccFX + g[0] * mass;
    gAccY = parentAccFY + g[1] * mass;
    gAccT = parentAccT
            + ((2 * AA + L * expansion) * cos(theta + PI) * parentAccFY
            - (2 * AA + L * expansion) * sin(theta + PI) * parentAccFX
            + (2 * AA + L * expansion) / 2 * cos(theta + PI) * g[1] * mass
            - (2 * AA + L * expansion) / 2 * sin(theta + PI) * g[0] * mass) * P2M_SCALE;
    /*cout << "gAccT = parentT + ((2 * AA + L * expansion) * cos(theta + PI) * parentY - (2 * AA + L * expansion) * sin(theta + PI) * parentX + (2 * AA + L * expansion) / 2 * cos(theta + PI) * g[1] * mass - (2 * AA + L * expansion) / 2 * sin(theta + PI) * g[0] * mass) * P2M_SCALE;" << endl;
    cout << gAccT << " = " << parentT << " + ((2 * " << AA << " + " << L << " * " << expansion << ") * cos(" << theta << " + " << PI << ") * " << parentY << " - (2 * " << AA << " + " << L << " * " << expansion << ") * sin(" << theta << " + " << PI << ") * " << parentX << " + (2 * " << AA << " + " << L << " * " << expansion << ") / 2 * cos(" << theta << " + " << PI << ") * " << g[1] << " * " << mass << " - (2 * " << AA << " + " << L << " * " << expansion << ") / 2 * sin(" << theta << " + " << PI << ") * " << g[0] << " * " << mass << ") * " << P2M_SCALE << endl;
    cout << gAccT << " = " << parentT << " + (" << (2 * AA + L * expansion) * cos(theta + PI) << " * " << parentY << " - " << (2 * AA + L * expansion) * cos(theta + PI) << " * " << parentX << " + " << (2 * AA + L * expansion) / 2 * cos(theta + PI) << " * "<< g[1] << " * " << mass << " - " << (2 * AA + L * expansion) / 2 * sin(theta + PI) << " * " << g[0] << " * " << mass << ") * " << P2M_SCALE << endl;*/
  }

  void getQP(double mass, double parentFX = 0.0, double parentFY = 0.0, double parentT = 0.0){ // the torque is about the end of this LINK
    GRAVITYDIR;
    double newFX = 0.0, newFY = 0.0, newT = 0.0;
    if (brace){
      newFX = forces.rbegin()[1][0] * forces.rbegin()[1][4]
              + forces.back()[0] * forces.back()[4];
      newFY = forces.rbegin()[1][1] * forces.rbegin()[1][4]
              + forces.back()[1] * forces.back()[4];

              // from normal force acting on current link
      newT = (forces.rbegin()[1][2] - getHeadX()) * forces.rbegin()[1][1] * forces.rbegin()[1][4]
              - (forces.rbegin()[1][3] - getHeadY()) * forces.rbegin()[1][0] * forces.rbegin()[1][4];
              // from friction acting on current link
              + (forces.back()[2] - getHeadX()) * forces.back()[1] * forces.back()[4]
              - (forces.back()[3] - getHeadY()) * forces.back()[0] * forces.back()[4];
    }
    QPfx = parentFX + g[0] * mass + newFX;
    QPfy = parentFY + g[1] * mass + newFY;
    QPt = parentT
            // from parent force
            + ((2 * AA + L * expansion) * cos(theta + PI) * parentFY
            - (2 * AA + L * expansion) * sin(theta + PI) * parentFX
            // from gravity of current link
            + (2 * AA + L * expansion) / 2 * cos(theta + PI) * g[1] * mass
            - (2 * AA + L * expansion) / 2 * sin(theta + PI) * g[0] * mass
            + newT
            ) * P2M_SCALE;

    // cout << "brace " << (brace? "true" : "false")
    //     << (brace? ", newFX " + to_string(newFX) + " N": "")
    //     << (brace? ", newFY " + to_string(newFY) + " N": "")
    //     << (brace? ", newT " + to_string(newT * P2M_SCALE) + " Nm": "") << endl;
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

#endif
