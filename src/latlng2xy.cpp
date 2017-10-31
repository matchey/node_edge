// (latitude, longitude) --> (x, y)
// 狭い範囲限定 経線を並行と近似して
#include <ros/ros.h>
#include <fstream>
#include "latlng2xy/latlng2xy.h"
#include "mstring/mstring.h"

using namespace std;

LatLng2xy::LatLng2xy()
	: lat1(0.0), lng1(0.0), lat2(0.0), lng2(0.0), x(0.0), y(0.0)
{
	// pub = n.advertise<>("", 1);
}

void LatLng2xy::set1(double lat, double lng){ lat1 = lat; lng1 = lng;} 

void LatLng2xy::set2(double lat, double lng){ lat2 = lat; lng2 = lng;}

void LatLng2xy::get(double &_x, double &_y){_x=x; _y=y;}

void LatLng2xy::calc()
{
	const double r = 6378137; //[m] 地球を球体としてその半径
	x = r * (lng2-lng1) * cos( (lat1+lat2)/2 );
	y = r * (lat2-lat1);
}

void LatLng2xy::calc(double (&latlng1)[2], double (&latlng2)[2], double (&xy)[2])
{
	const double r = 6378137; //[m] 地球を球体としてその半径
	// double yaw = 0.0;
	double lat1 = latlng1[0] * M_PI/180;
	double lng1 = latlng1[1] * M_PI/180;
	double lat2 = latlng2[0] * M_PI/180;
	double lng2 = latlng2[1] * M_PI/180;

	double dlat = lat2 - lat1; // 緯度の差
	double dlng = lng2 - lng1; // 経度の差
	double dx = r * dlng * cos( (lat1+lat2)/2 );
	double dy = r * dlat;
	double dist = sqrt(dx*dx + dy*dy);
	// yaw = atan2(dy, dx);

	xy[0] = dx;
	xy[1] = dy;
}

int LatLng2xy::writeCSV(const string ifilename, const string ofilename)
{
	static double x_last = 0.0;
	static double y_last = 0.0;
	ifstream ifs(ifilename);
	ofstream ofs(ofilename); // c++11でなければchar[]で渡す
	string str;
	vector<string> v;
	int cnt = 0;

	if(ifs.fail()){
		cerr << "can't read " << ifilename << endl;
		return -1;
	}

	while(getline(ifs, str)){
		if(!str.empty()){
			v = split(str, ' ');
			if(cnt){
				lat2 = atof(v[0].c_str()) * M_PI/180;
				lng2 = atof(v[1].c_str()) * M_PI/180;
				calc();
				ofs << "EDGE " << cnt-1 << " " << cnt << " " << atan2(y-y_last, x-x_last) << endl;
				ofs << "VERTEX " << cnt << " " << x << " " << y << endl;
				x_last = x;
				y_last = y;
			}else{
				lat1 = atof(v[0].c_str()) * M_PI/180;
				lng1 = atof(v[1].c_str()) * M_PI/180;
				ofs << "VERTEX " << 0 << " " << 0.000 << " " << 0.000 << endl;
			}
			cnt++;
		}
	}
	ofs.close();

	return 0;
}

void LatLng2xy::pub4vis()
{
}

