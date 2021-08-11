#include <ndt_localization/ndt_localization.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ndt_localization");

	NDTLocalization ndt_localization;

	ros::spin();

	return 0;
}
