#include <ros/ros.h>
#include "spline.h"

class ros_main{
       public:
    ros_main() : 
        nh{"~"},
        limiter(10),
        speed(1.5),
        mult(0.5),
        mult_accumulate(0.0),
        accumulate(0.0),
        reverse(false)
    {   

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

        std::vector<spline> splines;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spline_follow");
    ros_main rs;

    ros::Rate r(10); 

    while (ros::ok()){
        if (rs.hasSub()){
            if (!rs.isEnabled()){
                rs.startup();
            } else {
                rs.run():
            }
        } else {
            if (rs.isEnabled()){
                rs.shutdown();
            }
        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}