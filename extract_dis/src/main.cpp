#include "extractDis.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ExtractDistance");
    ExtractDistance ED;
    while(true){
        ED.run();
        ros::spinOnce();
    }

    return 0;
}
