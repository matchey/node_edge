// EDGE n m --> EDGE n m 1.322
// calc yaw
#include <ros/ros.h>
#include "mstring/mstring.h"
#include <fstream>

using namespace std;

struct Node
{
	double x;
	double y;
};

class YawCalculater
{
	ros::NodeHandle n;
	// ros::Publisher pub;
	
	double x1, y1, x2, y2;
	double yaw;
	map<int, Node> nodes; // < node_num, (x, y) >

	public:
	YawCalculater()
		: x1(0.0), y1(0.0), x2(0.0), y2(0.0), yaw(0.0)
	{
		// pub = n.advertise<>("", 1);
	}

	void set1(double x, double y){ x1 = x; y1 = y;} 
	void set2(double x, double y){ x2 = x; y2 = y;}
	void calc();
	int writeCSV(const string ifilename, const string ofilename);
};

void YawCalculater::calc()
{
	yaw = atan2(y2-y1, x2-x1);
}

int YawCalculater::writeCSV(const string ifilename, const string ofilename)
{
	ifstream ifs(ifilename);
	ofstream ofs(ofilename); // c++11でなければchar[]で渡す
	string str;
	vector<string> v;
	Node node;

	if(ifs.fail()){
		cerr << "can't read " << ifilename << endl;
		return -1;
	}

	while(getline(ifs, str)){
		if(!str.empty()){
			v = split(str, ' ');
			if(v[0] == "VERTEX"){
				node.x =atof(v[2].c_str());
				node.y =atof(v[3].c_str());
				nodes[atoi(v[1].c_str())] = node;
			}
		}
	}
	for(auto itr = nodes.begin(); itr != nodes.end(); ++itr) {
		ofs << "VERTEX " << itr->first << " " << itr->second.x << " " << itr->second.y << endl;
		// std::cout << "key = " << itr->first           // キーを表示
		// 	<< ", val = " << itr->second << "\n";    // 値を表示
	}
	ifs.clear();
	ifs.seekg(0,ios_base::beg);
	while(getline(ifs, str)){
		if(!str.empty()){
			v = split(str, ' ');
			if(v[0] == "EDGE"){
				x1 = nodes[atof(v[1].c_str())].x;
				y1 = nodes[atof(v[1].c_str())].y;
				x2 = nodes[atof(v[2].c_str())].x;
				y2 = nodes[atof(v[2].c_str())].y;
				calc();
				// ofs << "VERTEX " << v[1] << " " << x1 << " " << y1 << endl;
				ofs << "EDGE " << v[1] << " " << v[2] << " " << yaw << endl;
				// ofs << "VERTEX " << v[2] << " " << x2 << " " << y2 << endl;
			}
		}
	}
	ofs.close();

	return 0;
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "edge2yaw");

	YawCalculater yc;

	yc.writeCSV("xy.csv", "xy_new.csv");

	return 0;
}

