
#include <ros/ros.h>

using namespace std;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "edge_atan2");

	double x1, x2, y1, y2;

	cout << "x1: " ; cin >> x1;
	cout << "y1: " ; cin >> y1;
	cout << "x2: " ; cin >> x2;
	cout << "y2: " ; cin >> y2;

	cout << atan2(y2-y1, x2-x1) << endl;

	return 0;
}
