#include <disturbance_observer/nonLinearDO.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "NDO");
	ros::NodeHandle n;
	nonLinearDO ndo(n);

	ros::spin();


	return 0;
}