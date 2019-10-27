#include <ros/ros.h>
#include <piksi_rtk_msgs/VelNed.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Twist.h>
#include <piksi_rtk_msgs/VelNed.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>

#include <vector>
#include <algorithm>
#include <cmath>

#include "point.h"
namespace spline{

    class spline{
        public:
            //generates a spline using the position given and the next three points
            spline(const PointsContainer::iterator& points_start);
            point get_point(const float& t);
            point get_first_dir(const float& t);
            point get_second_dir(const float& t);
            float find_closest_time(const point& cur);
            point find_closest_point(const point& cur);
            float get_length();

        private:
            PointsContainer coeff;
            PointsContainer points;
            float length; 

            static float length_increments = 0.01;
            static float closeness_resolution = 0.01;
            static float distance_scale = 0.5;
            static float spline_scale = 0.5;
    };
}