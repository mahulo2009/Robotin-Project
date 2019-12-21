#include "robotin_base.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "robotin_base_node");
    RobotinBase robotin;
    ros::spin();
    return 0;
}
