
#ifndef EDGE2POINT_H
#define EDGE2POINT_H

struct SegPoint
{
	double x;
	double y;
	int edge;
};

namespace edge2point{

struct Node
{
	double x;
	double y;
};

struct Edge
{
	int begin;
	int end;
	double orientation;
};

class Edge2point{
	ros::NodeHandle n;
	ros::Publisher marker_pub;
	ros::Publisher labels_pub;
	std::vector<Node> nodes;
	std::vector<Edge> edges;
	double dist_min;
	std::vector<SegPoint> seg_points;

	double getDist(int);
	void setMinDist();
	void pubPoints();

	public:
	void pubPoints(std::vector<SegPoint> &points);
	Edge2point();
	void loadCSV(std::string filename);
	void getMinDist(double&);
	void getMinXY(double&, double&); //x, y
	void getMapSize(double&, double&); //x, y
	void getEdgeYaws(std::vector<double>&); //x, y
	void createPoints();
	void createPoints(std::vector<SegPoint>&);
};

}

#endif

