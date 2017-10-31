//
// xy.csvを読み込みnode-edgeをrvizに表示
//

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "mstring/mstring.h"

#include <fstream>
// #include<iostream>

using namespace std;

string filename = "/home/amsl/ros_catkin_ws/src/mapping/latlng2xy/xy.csv";

bool set_points(visualization_msgs::Marker* points, visualization_msgs::Marker* line_strip, visualization_msgs::Marker* line_list, visualization_msgs::MarkerArray &labels)
{
	// ifstream ifs("xy.csv");
	ifstream ifs(filename);
	string str;
	vector<string> v;
	int cnt = 0;

	if (ifs.fail())
	{
		cerr << "can't open " << "file" << endl;
		return false;
	}
	visualization_msgs::Marker label;
	label.header.frame_id = "map";
	// label.header.stamp = ros::Time::now();
	label.ns = "nodelabels";
	label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	label.action = visualization_msgs::Marker::ADD;
	// label.scale.z = 8.35;
	label.scale.z = 5.35;
	label.color.a = 1.0;
	label.color.r = 1.0;
	label.color.g = 0.5;
	label.color.b = 0.5;
	while (getline(ifs, str) )
	{
		v=split(str,' ');

		if(v[0] == "VERTEX"){
			geometry_msgs::Point p;
			p.x = label.pose.position.x = atof(v[2].c_str());
			p.y = label.pose.position.y = atof(v[3].c_str());
			p.z = -0.10;
			label.pose.position.z = -0.05;
			label.text = v[1];
			label.id = atoi(v[1].c_str());

			points->points.push_back(p);
			// line_strip->points.push_back(p); // 全部つなげるとき(wayoiintみたいな)
			labels.markers.push_back(label);
			cnt++;
		}
	}
	ifs.clear();
	ifs.seekg(0,ios_base::beg);
	while (getline(ifs, str) )
	{
		v=split(str,' ');
		if(v[0] == "EDGE"){
			// cout << atoi(v[1].c_str()) << ", " << atoi(v[2].c_str()) << endl;
			line_list->points.push_back(points->points[atoi(v[1].c_str())]);
			line_list->points.push_back(points->points[atoi(v[2].c_str())]);
		}
	}

	return true;
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "show_node");
	ros::NodeHandle n;
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/map/node_edge", 1);
	ros::Publisher labels_pub = n.advertise<visualization_msgs::MarkerArray>("/labels/node", 1);

	ros::Rate r(10);

	n.getParam("/node_edge", filename);

	bool flag_paper = false;
	if(argc == 2){
		if(strcmp(argv[1], "paper")){
			filename = argv[1];
		}else{
			flag_paper = true;
		}

	}

    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "map";
    points.ns = line_strip.ns = line_list.ns = "points_and_lines";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

	visualization_msgs::MarkerArray labels;


    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;



    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;


    // POINTS markers use x and y scale for width/height respectively
    // points.scale.x = 0.5; //for d_in
    // points.scale.y = 0.5; //for d_in
    // points.scale.x = 0.9;
    // points.scale.y = 0.9;
    // points.scale.x = 1.1;
    // points.scale.y = 1.1;
    points.scale.x = 2.1;
    points.scale.y = 2.1;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    // line_strip.scale.x = 0.1;
    // line_list.scale.x = 0.1; //for d_in
    // line_strip.scale.x = 0.1;
    // line_list.scale.x = 0.1;
    // line_strip.scale.x = 0.8;
    line_list.scale.x = 1.8;



    // Points are green
    points.color.g = 1.0f;
    points.color.a = 0.6;

    // Line strip is blue
    // line_strip.color.b = 1.0;
    // line_strip.color.a = 1.0;

    // // Line list is red
    // line_list.color.r = 1.0;
    // line_list.color.a = 1.0;
	
	
    // Line list is blue
    // line_list.color.a = 1.0;
    if(!flag_paper) line_list.color.b = 1.0;

    // Line strip is grey for paper
    // line_list.color.a = 1.0;
    line_list.color.a = 0.4;



    // Create the vertices for the points and lines
	set_points(&points, &line_strip, &line_list, labels);

  while (ros::ok())
  {
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
	for(unsigned int i = 0; i < labels.markers.size(); i++){
		labels.markers[i].header.stamp = ros::Time::now();
	}
    // marker_pub.publish(line_strip);
    marker_pub.publish(points);
    marker_pub.publish(line_list);
    labels_pub.publish(labels);

    r.sleep();
  }
}

