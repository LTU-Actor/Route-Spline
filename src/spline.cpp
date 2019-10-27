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

        spline::Spline(const PointsContainer::iterator& points_start){
            coeff = PointsContainer();
            points = PointsContainer();
            std::copy_n(points_start, 4, points);

            //hermite functions
            coeff.emplace_back(0.5*((-1.0f*points[0]) + (3.0f*points[1]) - (3.0f*points[2]) + (points[3])));
            coeff.emplace_back(0.5*( (2.0f*points[0]) - (5.0f*points[1]) - (4.0f*points[2]) - (points[3])));
            coeff.emplace_back(0.5*((-1.0f*points[1]) + (points[2])));

            length = 0;

            float j = 0;

            for (float f = length_increments; f < 1; f += length_increments ){
                length += distance(this->get_point(f), this->get_point(j));
            }

        }

        point spline::get_point(const float& t){
            return ((((((coeff[0]*t) + coeff[1])*t) + coeff[2])*t )+ points[0]);
        }

        point spline::get_first_dir(const float& t){
            return (((3*coeff[0]*t) + (2*coeff[1]))*t + coeff[2]);
        }

        point spline::get_second_dir(const float& t){
            return ((6*coeff[0]*t )+ (2*coeff[1]));
        }

        float spline::find_closest_time(const point& cur){
            float t = 0.5;
            //spline function and its dirivatives
            point f, fd, fdd;
            //distance function and its dirivatives
            float d, dd, ddd;

            //first dirivative of distance
            auto distance_dir = [&](){
                point temp = fd * (f - cur);
                return (temp.x + temp.y)/d;
            };

            //second dirivative of distance
            auto distance_second_dir = [&](){
                point temp = (fdd*(f - cur)) + (fd*fd);
                point temp2 = (f - cur)*fd;
                float sum2 = (temp2.x + temp2.y);
                float part1 = (temp.x + temp.y)/d;
                float part2 = (sum2*sum2)/(d*d*d);
                return part1 + part2;
            };

            do{
                f = this->get_point(t);
                fd = this->get_first_dir(t);
                fdd = this->get_second_dir(t);

                d = distance(cur, f);
                dd = distance_dir();
                ddd = distance_second_dir();

                t = t - (dd/ddd);
            } while(abs(dd) > closeness_resolution && t >= 0 && t < 1);

            if (t < 0)
                return 0;
            
            if (t > 1)
                return .9999;

            return t;
        }

        spline::point find_closest_point(const point& cur){
            return get_point(find_closest_time(cur));
        }

        spline::float get_length(){
            return length;
        }

        private:
            PointsContainer coeff;
            PointsContainer points;
            float length; 
    };
}