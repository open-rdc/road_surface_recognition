#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

ros::Subscriber LaserScan_subscriber;

void get_scan_value(const sensor_msgs::LaserScan::ConstPtr& scan);


int main(int argc, char **argv)
{
    //Initiate new ROS node named "get_scan_value"
    ros::init(argc, argv, "get_scan_value");
    ros::NodeHandle n;

    ROS_INFO("     ************** Start reading scan value of 2D LIDAR **************\n");
    LaserScan_subscriber = n.subscribe<sensor_msgs::LaserScan>("/diag_scan", 10, get_scan_value);

    ros::spin();
    return 0;
}


void get_scan_value(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    const int sensor_pitch_deg = 17;
    const double PI = 3.14159;
    int    laser_sumNum   = scan->ranges.size();
    double range     = scan->ranges[laser_sumNum/2];
    double intensity = scan->intensities[laser_sumNum/2];
    double sensor_hight      = scan->ranges[laser_sumNum/2] * sin(sensor_pitch_deg * PI /180);
    double d_R2MLine = scan->ranges[laser_sumNum/2] * cos(sensor_pitch_deg * PI /180);
    double rad_MLine = asin(scan->ranges[laser_sumNum/2]/scan->ranges[1]);


    printf("l_sumNum:%d  ", laser_sumNum);
    printf("s_int:%f  ",intensity); // sentor of intensity
    printf("s_ran:%f  ",range); // sentor of range
    printf("f_ran:%f  ",scan->ranges[1]); // distance of measurement line
    printf("h_sen:%f  ", sensor_hight); // sensor hight
    printf("d_R2MLine:%f  ", d_R2MLine); // distance between robot and measurement line
    printf("d_MLine:%f\n", 2 * scan->ranges[1] * cos(rad_MLine)); // distance measurement line

}
