#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <sys/time.h>
/* Setting for transforming LaserScan into PointCloud */
#include <laser_geometry/laser_geometry.h>
/* Setting for PCL library */
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
/* Setting for convertPointCloud2ToPointCloud */
#include <sensor_msgs/point_cloud_conversion.h>


class Making_Envir_Cloud
{
    public:
        Making_Envir_Cloud();

    private:
        ros::NodeHandle nh;
        ros::Subscriber diag_scan_sub;
        ros::Publisher  disting_cloud_pub;

        tf::TransformListener listener;
        std::string error_msg;
        laser_geometry::LaserProjection projector;

        sensor_msgs::PointCloud2 cloud_in;
        sensor_msgs::PointCloud2 disting_cloud2;

        void diagScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in);
};


Making_Envir_Cloud::Making_Envir_Cloud()
{
    diag_scan_sub   = nh.subscribe<sensor_msgs::LaserScan>("/diag_scan", 100, &Making_Envir_Cloud::diagScanCallback, this);
    disting_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/disting_cloud2", 100, false);
}


void Making_Envir_Cloud::diagScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    struct timeval s, e;
    ros::Rate loop_rate(10); // 10 = 100ms
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);

    gettimeofday(&s, NULL);


    /* Transform diagonally_hokuyo_link frame to map frame */
    if(listener.waitForTransform(
                scan_in->header.frame_id, 
                "/map",
                scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
                ros::Duration(1.0),
                ros::Duration(0.01),
                &error_msg ) == 0){
        std::cout <<"Failure in waitForTransform" << std::endl;
        ROS_WARN("%s", error_msg.c_str());
        return;
    }


    /* Transform LaserScan msg to PointCloud2 msg */
    try{
        projector.transformLaserScanToPointCloud("/map", *scan_in, cloud_in, listener);
    }catch(tf::TransformException e){
        ROS_WARN("Could not transform scan to pointcloud! (%s)", e.what());
        return;
    }

    /* Convert PointCloud2 field[3].name to PointXYZI intensity field name */
    cloud_in.fields[3].name = "intensity";
    pcl::fromROSMsg(cloud_in, *raw_cloud);

    copyPointCloud(*raw_cloud, *filtered_cloud);


    /* Saving LaserScan data processing */
    static int i = 1;
    char buf[10];
    std::string scan_file_path = "/home/kenta/pcd/making_envir_cloud/laserscan_data/";
    std::string scan_file_name;

    sprintf(buf, "%d", i);
    scan_file_name.append(scan_file_path);
    scan_file_name.append(buf);
    scan_file_name.append(".txt");
    std::ofstream ofs(scan_file_name.c_str(), std::ofstream::out);

    for(int i = 0; i < scan_in->ranges.size(); i++)
        ofs << scan_in->ranges[i] << "\t" << scan_in->intensities[i] << std::endl;
    ofs.close();


    /* Saving raw_cloud processing */
    std::string raw_file_path = "/home/kenta/pcd/making_envir_cloud/raw_data/";
    std::string raw_file_name;
    raw_file_name.append(raw_file_path);
    raw_file_name.append(buf);
    raw_file_name.append(".pcd");
    pcl::io::savePCDFileASCII (raw_file_name, *raw_cloud);


    /* If flag is true, detecting low level is on. Flag is initialized to false. */
    double a, b; //for rms error function
    int flag = false;
    if(flag == true){
        double sum_z, sum_y, sum_y2, sum_yz;
        const int N = 100;
        sum_z = sum_y = sum_y2 = sum_yz = 0;

        // Calculate an inclination of measurement line from center laser values.
        for(int i = 200; i <= 299 ; i++){
            sum_y += raw_cloud->points[i].y;
            sum_z += raw_cloud->points[i].z;
            sum_yz += raw_cloud->points[i].y * raw_cloud->points[i].z;
            sum_y2 += raw_cloud->points[i].y * raw_cloud->points[i].y;
        }
        a = (N * sum_yz - sum_y * sum_z) / (N * sum_y2 - pow(sum_y,2));
        b = (sum_y2 * sum_z - sum_yz * sum_y) / (N * sum_y2 - pow(sum_y,2));

        // If Calculated inclination is over threshold value, process is stopped because got scan values include obstacle. 
        if(a <= -0.025 || a >= 0.025)
            return ;
    }

    /* Detect low level processing and distinguish cloud processing */
    for(int i = 0; i < filtered_cloud->points.size(); i++){
        double normaliz = scan_in->intensities[i] / (48.2143 * scan_in->ranges[i] * scan_in->ranges[i] - 840.393 * scan_in->ranges[i] + 4251.14+300+40);
        //double normaliz = scan_in->intensities[i] / (-430 * scan_in->ranges[i] + 3900);
        
        if(flag == true){
            if(filtered_cloud->points[i].z >= a * filtered_cloud->points[i].y + b + 0.038){
                filtered_cloud->points[i].intensity = 100.0;
            }else{
              if(normaliz >= 1)
                  filtered_cloud->points[i].intensity = 100.0;
              else
                  filtered_cloud->points[i].intensity = 0.1;
            }
        }else{
            if(normaliz >= 1)
                filtered_cloud->points[i].intensity = 100.0;
            else
                filtered_cloud->points[i].intensity = 0.1;
        }
    }


    /* Saving filtered_cloud processing */
    std::string filtered_file_path = "/home/kenta/pcd/making_envir_cloud/filtered_data/";
    std::string filtered_file_name;
    filtered_file_name.append(filtered_file_path);
    filtered_file_name.append(buf);
    filtered_file_name.append(".pcd");
    pcl::io::savePCDFileASCII (filtered_file_name, *filtered_cloud);
    i++;

    /* Publish PointCloud2 which has distinguished clouds */
    toROSMsg (*filtered_cloud, disting_cloud2);
    disting_cloud_pub.publish(disting_cloud2);
    std::cout << "Success in publishing   ";

    loop_rate.sleep();
    gettimeofday(&e, NULL);
    std::cout << "Compute time: " <<  std::setw(9) << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6 << "s" << std::endl;
}


int main(int argc, char **argv)
{
    /* Initiate new ROS node named "making_envir_cloud" */
    ros::init(argc, argv, "making_envir_cloud");

    ROS_INFO("\n************** Start this program **************\n");
    Making_Envir_Cloud making_envir_cloud;

    ros::spin();
    return 0;

}

