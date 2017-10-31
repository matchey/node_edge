#include <ros/ros.h>
#include <fstream> // <- ないとifstream でincomplete typeってerrorでる
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "node_edge/edge2point.h"
#include "mstring/mstring.h"

using namespace std;
using namespace edge2point;

Edge2point::Edge2point()
{
	marker_pub = n.advertise<visualization_msgs::Marker>("/edge/segpoints", 1);
	labels_pub = n.advertise<visualization_msgs::MarkerArray>("/segedge/labels", 1);
}

void Edge2point::loadCSV(string filename)
{
	Node node;
	Edge edge;
	ifstream ifs(filename);
	string str;
	vector<string> v;

	if(ifs.fail()){
		cerr << "can't read " << filename << endl;
		return;
	}
	// cout << "file name: " << filename << endl;

	while(getline(ifs, str)){
		if(!str.empty()){
			v = split(str, ' ');
			if(v[0] == "VERTEX"){
				node.x = atof(v[2].c_str());
				node.y = atof(v[3].c_str());
				nodes.push_back(node);
			}else if(v[0] == "EDGE"){
				edge.begin = atoi(v[1].c_str());
				edge.end = atoi(v[2].c_str());
				edge.orientation = atof(v[3].c_str());
				edges.push_back(edge);
				edge.begin = atoi(v[2].c_str());
				edge.end = atoi(v[1].c_str());
				edge.orientation += (edge.orientation > 0 ? -1 : 1) * M_PI;
				edges.push_back(edge);
			}else{
				cout << "cannot read:\n\t" << str << endl;
			}
		}
	}
	setMinDist();
}

double Edge2point::getDist(int n)
{
	return sqrt(pow(nodes[edges[n].begin].x - nodes[edges[n].end].x, 2)
			+ pow(nodes[edges[n].begin].y - nodes[edges[n].end].y, 2));
}

void Edge2point::setMinDist()
{
	double min = getDist(0);
	double dist = 0;
	for(int i = 1; i < (int)edges.size(); i++){
		dist = getDist(i);
		if(dist < min){
			min = dist;
		}
	}
	dist_min = min;
}

void Edge2point::getMinDist(double &min)
{
	min = dist_min;
}

void Edge2point::getMinXY(double &x_min, double &y_min)
{
	for(auto node = nodes.begin(); node != nodes.end(); ++node){
		if(x_min > node->x) x_min = node->x;
		if(y_min > node->y) y_min = node->y;
	}
}

void Edge2point::getMapSize(double &len_x, double& len_y)
{
	double x_max = 0;
	double y_max = 0;
	double x_min = 0;
	double y_min = 0;
	for(auto node = nodes.begin(); node != nodes.end(); ++node){
		if(x_max < node->x) x_max = node->x;
		if(y_max < node->y) y_max = node->y;
		if(x_min > node->x) x_min = node->x;
		if(y_min > node->y) y_min = node->y;
	}
	len_x = x_max - x_min;
	len_y = y_max - y_min;
}

void Edge2point::getEdgeYaws(vector<double> &v)
{
	for(auto edge = edges.begin(); edge != edges.end(); ++edge){
		v.push_back(edge->orientation);
	}
}

void Edge2point::createPoints()
{
	double div_dist = dist_min / 2.0;
	SegPoint sp;
	for(int i = 0; i < (int)edges.size(); i++){
		sp.edge = i;
		for(int j = 0; j*div_dist <= getDist(i); j++){
			sp.x = nodes[edges[i].begin].x + j*div_dist * cos(edges[i].orientation);
			sp.y = nodes[edges[i].begin].y + j*div_dist * sin(edges[i].orientation);
			seg_points.push_back(sp);
		}
		sp.x = nodes[edges[i].end].x;
		sp.y = nodes[edges[i].end].y;
		seg_points.push_back(sp);
	}
}

void Edge2point::createPoints(std::vector<SegPoint> &points)
{
	double div_dist = dist_min / 2.0;
	SegPoint sp;
	for(int i = 0; i < (int)edges.size(); i++){
		sp.edge = i;
		for(int j = 0; j*div_dist <= getDist(i); j++){
			sp.x = nodes[edges[i].begin].x + j*div_dist * cos(edges[i].orientation);
			sp.y = nodes[edges[i].begin].y + j*div_dist * sin(edges[i].orientation);
			points.push_back(sp);
		}
		sp.x = nodes[edges[i].end].x;
		sp.y = nodes[edges[i].end].y;
		points.push_back(sp);
	}
}

void Edge2point::pubPoints()
{
    visualization_msgs::Marker points;
    points.header.frame_id = "/map";
    points.ns = "/edge/points";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;

    points.type = visualization_msgs::Marker::POINTS;

	points.color.r = 1.0f;
	// points.color.g = 1.0f;
	// points.color.b = 1.0f;
	points.color.a = 1.0;

	points.header.stamp = ros::Time::now();

	geometry_msgs::Point p;
	
	points.scale.x = 1.8;
	points.scale.y = 1.8;

	for(auto sp = seg_points.begin(); sp != seg_points.end(); ++sp){
		p.x = sp->x;
		p.y = sp->y;
		points.points.push_back(p);
	}
		
	marker_pub.publish(points);
}

void Edge2point::pubPoints(std::vector<SegPoint> &spoints)
{
    visualization_msgs::Marker points;
	visualization_msgs::MarkerArray labels;

    points.header.frame_id = "/map";
    points.ns = "/edge/segpoints";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;

    points.type = visualization_msgs::Marker::POINTS;

	// points.color.r = 1.0f;
	points.color.g = 1.0f;
	// points.color.b = 1.0f;
	points.color.a = 1.0;

	points.scale.x = 1.1;
	points.scale.y = 1.1;

	points.header.stamp = ros::Time::now();


	visualization_msgs::Marker label;
	label.header.frame_id = "/map";
	// label.header.stamp = ros::Time::now();
	label.ns = "seglabels";
	label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	label.action = visualization_msgs::Marker::ADD;
	label.scale.z = 1.4;
	label.color.a = 1.0;
	label.color.r = 0.5;
	label.color.g = 0.5;
	label.color.b = 1.5;

	geometry_msgs::Point p;
	
	p.z = 0.0;
	for(auto sp = spoints.begin(); sp != spoints.end(); ++sp){
		p.x = label.pose.position.x = sp->x;
		p.y = label.pose.position.y = sp->y;
		label.text = to_string(sp->edge);
		label.id = (sp - spoints.begin());
		// cout << "(" << p.x << ", " << p.y << ")" << endl;
		points.points.push_back(p);
		if(label.id < 500)
		labels.markers.push_back(label);
	}

	// marker_pub.publish(points);
	labels_pub.publish(labels);
}

