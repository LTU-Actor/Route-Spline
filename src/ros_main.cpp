#include <ros/ros.h>
#include "spline.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spline_follow");
    spline spl;

    ros::Rate r(10); 

    while (ros::ok()){
        if (spl.hasSub()){
            if (!spl.isEnabled()){
                spl.startup();
            } else {
                spl.run():
            }
        } else {
            if (spl.isEnabled()){
                spl.shutdown();
            }
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}