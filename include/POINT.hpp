#ifndef POI
#define POI

// standard headers
// #include <stdio.h>
// #include <vector>
// #include <math.h>

// DOSL headers:
// #include <dosl/dosl>
// #include "parameters.hpp"
// using namespace std;

class POINT : public AStar::Node<POINT,double>{
public:
	double x, y;
  double distanceToEnd;
	POINT() {	};
	POINT(double x, double y)
	{
		this->x = x;
		this->y = y;
    this->distanceToEnd = 10000.0;
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

  double getDistance(const POINT & n) const {
    return sqrt(pow(x - n.x, 2.0) + pow(y - n.y, 2.0));
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

#endif