#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <std_srvs/Empty.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    //Converting ROSmsg to pPCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*msg, *tmp_cloud);
    //Filtering input cloud
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (tmp_cloud);
    sor.setLeafSize (0.5, 0.5, 0.5); //50cm filter
    sor.filter (*tmp_cloud);
    //Add new input to previus cloud
    *cloud = *cloud + *tmp_cloud;
}

bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  cloud->clear();
  ROS_INFO("Point Cloud Map");
  return true;
}

int main(int argc, char** argv){
    //Initialize ROS
    ros::init (argc,argv,"map_pcl");
    ros::NodeHandle nh;
    //Set up subscriber and Publisher
    ros::Subscriber velodyne_subscriber;
    velodyne_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, velodyne_callback);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_points",1);
    ros::ServiceServer resetService = nh.advertiseService("/map_pcl/reset", resetSrv);
    //Setting rate of one second
    ros::Rate rate(1);

    while(ros::ok()){
        // Create the filtering object and perform filtering
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.5, 0.5, 0.5); //50cm filter
        sor.filter (*cloud);        

        //Create new message, convert it and send it
        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2Ptr cloud_tmp(new pcl::PCLPointCloud2());
        pcl::toPCLPointCloud2(*cloud, *cloud_tmp);
        pcl_conversions::fromPCL(*cloud_tmp, output);

        pub.publish(output);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}