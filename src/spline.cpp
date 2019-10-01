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



float length_increments = 0.01;
float closeness_resolution = 0.01;
float distance_scale = 0.5;
float spline_scale = 0.5;

struct point{
    point(): x(0), y(0) {}

    point(const float& x, const float& y): x(x), y(y) {}

    float x;
    float y;
};

point operator*(const point& p1, const point& p2){
    return {p1.x * p2.x, p1.y * p2.y};
}

point operator+(const point& p1, const point& p2){
    return {p1.x + p2.x, p1.y + p2.y};
}

point operator-(const point& p1, const point& p2){
    return {p1.x - p2.x, p1.y - p2.y};
}

point operator*(const point& p, const float& f){
    return {p.x*f, p.y*f};
}

point operator*(const float& f, const point& p){
    return {p.x*f, p.y*f};
}

point operator/(const point& p, const float& f){
    return {p.x*f, p.y*f};
}

point operator/(const float& f, const point& p){
    return {p.x*f, p.y*f};
}

float magnitude(point p){
    return sqrt( (p.x*p.x) + (p.y*p.y) );
}

float distance(const point& p1, const point& p2){
    return  magnitude(p1 - p2);
}

using PointsContainer = std::vector<const point>;

class subSpline{
    public:
    //generates a spline using the position given and the next three points
    subSpline(const PointsContainer::iterator& points_start){
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

    point get_point(const float& t){
        return ((((((coeff[0]*t) + coeff[1])*t) + coeff[2])*t )+ points[0]);
    }

    point get_first_dir(const float& t){
        return (((3*coeff[0]*t) + (2*coeff[1]))*t + coeff[2]);
    }

    point get_second_dir(const float& t){
        return ((6*coeff[0]*t )+ (2*coeff[1]));
    }

    float find_closest_time(const point& cur){
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

    point find_closest_point(const point& cur){
        return get_point(find_closest_time(cur));
    }

    float get_length(){
        return length;
    }

    private:
        PointsContainer coeff;
        PointsContainer points;
        float length; 
};
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
class spline{
    public:
    spline() : 
        nh{"~"},
        limiter(10),
        speed(1.5),
        mult(0.5),
        mult_accumulate(0.0),
        accumulate(0.0),
        reverse(false)
    {   

        pos_sub_ = nh_.subscribe("")
        splines = std::vector<subSpline>();
        PointsContainer points;
        for (PointsContainer::iterator it = points.begin(); it != points.end()-3; ++it){
            splines.emplace_back(it);
        }

        // Global
        gps_sub         = nh.subscribe<sensor_msgs::NavSatFix>(gps_fix_topic,  4, &GotoWaypoint::gpsCallback, this);
        gps_velned_sub  = nh.subscribe<piksi_rtk_msgs::VelNed>(gps_vel_ned_topic, 4, &GotoWaypoint::gpsVelnedCallback, this);

        // Local
        waypoint_sub    = nh.subscribe<sensor_msgs::NavSatFix>("waypoint", 4, &GotoWaypoint::waypointCallback, this);
        debug_angle_pub = nh.advertise<std_msgs::Float64>("debug_angle", 1);
        twist_pub = nh.advertise<geometry_msgs::Twist>("cmd", 1);

        gps_fix.status.status = -1; // NO FIX


        // Load configuration params
        if (!nh_.getParam("gps_fix", gps_fix_topic)){
            ROS_ERROR_STREAM("Route-Spline: parameter 'gps_fix' not defined.")
            exit(0);
        }

        if (!nh_.getParam("gps_vel_ned",gps_velned_topic)){
            ROS_ERROR_STREAM("Route-Spline: parameter 'gps_vel_ned' not defined.")
            exit(0);
        }

        nh.getParam("speed", speed);
        nh.getParam("mult", mult);
        nh.getParam("mult_accumulate", mult_accumulate);
        nh.getParam("accumulate", accumulate);
        nh.getParam("reverse", reverse);
    }

    point getMoveVector(const point& cur){
        if (splines.size() < 1){
            return {0,0};
        }

        int closest_spline = 0;
        float closest_time = splines[closest_spline].find_closest_time(cur);
        float closest_distance = distance(cur, splines[closest_spline].get_point(closest_time));

        float temp_time;
        float temp_distance;

        for (int i = 1; i < splines.size(); ++i){
            temp_time = splines[i].find_closest_time(cur);
            temp_distance = distance(splines[i].get_point(temp_time), cur);
            if (temp_distance < closest_distance){
                closest_spline = i;
                closest_time = temp_time;
                closest_distance = temp_distance;
            }
        }

        point slope = splines[closest_spline].get_point(closest_time) - cur;

        point dir = splines[closest_spline].get_first_dir(closest_time);

        slope = slope + (dir/splines[closest_spline].get_length());
        
        return cur + slope;
    }

    void shutdown() {

        enabled =  false;
    }
    void startup(){
        
        enabled = true;
    }
    bool hasSub(){
        return twist_pub_.getNumSubscribers()> 0;
    }

    bool isEnabled(){
        return enabled;
    }

    void run(){
        geometry_msgs::Twist command;
        std_msgs::Float64 angle_msg;
        command.linear.x = speed;

        if (gps_fix.status.status == -1)
            ROS_ERROR_STREAM_THROTTLE(10, NODE_NAME ": No GPS Fix!");
        else if (gps_velned.n_sats < 2)
            ROS_ERROR_STREAM_THROTTLE(10, NODE_NAME ": Bad vel_ned (not enough sats)!");
        else
        {
            point cur{gps_fix.latitude, gps_fix.longitude};

            auto move_vector = getMoveVector(cur);
            float direction = atan2(gps_velned.n,gps_velned.e);

            float angle = atan2(cur.x, cur.y) - direction;

            if (std::isnan(angle)) angle = 0;

            angle_msg.data = angle;

            if (speed > 600) accumulate += angle * limiter.cycleTime().nsec/1000000000.0f * mult_accumulate;

            if (accumulate > 0.1) accumulate = 0.1;
            else if (accumulate < -0.1) accumulate = -0.1;

            command.angular.z = speed > 500 ? angle * mult : 0;

            if (!std::isnan(accumulate))
                command.angular.z += accumulate;

            float scale = dist * 4503.0f;
            scale *= scale;

            if (std::abs(command.angular.z) > scale)
                command.angular.z = copysignf(scale, command.angular.z);

            if (reverse) command.angular.z *= -1;

            twist_pub.publish(command);
            debug_angle_pub.publish(angle_msg);
        }
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) { gps_fix = *msg; }
    void waypointsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) { target = *msg; }
    void gpsVelnedCallback(const piksi_rtk_msgs::VelNed::ConstPtr& msg) { gps_velned = *msg; }

    private:
        bool enabled = false;
        ros::NodeHandle nh_;
        ros::Subscriber pos_sub_;
        ros::Publisher twist_pub_;

        sensor_msgs::NavSatFix gps_fix;
        piksi_rtk_msgs::VelNed gps_velned;

        double accumulate;
        double mult_accumulate;
        double mult;
        double speed;
        bool reverse;

        std::string gps_fix_topic;
        std::string gps_velned_topic;

        std::vector<subSpline> splines;
};

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