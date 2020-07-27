#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;
// pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
// pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
 
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // Container for original & filtered data
    // pcl::PCLPointCloud2* tmp_cloud(new pcl::PCLPointCloud2()); 
    // pcl::PCLPointCloud2ConstPtr tmp_cloudPtr(tmp_cloud);
    // pcl::PCLPointCloud2 cloud_filtered;
    // Convert to PCL data type
    // pcl_conversions::toPCL(*msg, *tmp_cloud);
    
    // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    // sor.setInputCloud (tmp_cloudPtr);
    // sor.setLeafSize (0.1, 0.1, 0.1);
    // sor.filter (cloud_filtered);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    
    // pcl::fromPCLPointCloud2 (cloud_filtered, tmp_cloud_filtered);
    // cloud += tmp_cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ::ConstPtr ptr_cloud(cloud);
    pcl::fromROSMsg (*msg, *tmp_cloud);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (tmp_cloud);
    sor.setLeafSize (0.5, 0.5, 0.5); //50cm filter

    sor.filter (*tmp_cloud);
    *cloud = *cloud + *tmp_cloud;

    
}

// void filter_map(){
//     // Create the filtering object
//     pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//     sor.setInputCloud (cloud);
//     sor.setLeafSize (0.01, 0.01, 0.01);
//     sor.filter (*cloud_filtered);
// }

int main(int argc, char** argv){
    //Initialize ROS
    ros::init (argc,argv,"map_pcl");
    ros::NodeHandle nh;
    ros::Subscriber velodyne_subscriber;
    velodyne_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, velodyne_callback);
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_points",1);
    ros::Rate(1);

    while(ros::ok()){
        // pcl::PCLPointCloud2* cloud_filtered;
        // Create the filtering object and perform filtering
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud);
        sor.setLeafSize (0.5, 0.5, 0.5); //50cm filter

        sor.filter (*cloud);        // sor.setInputCloud (cloudPtr);
        // sor.filter (cloud_filtered);

        sensor_msgs::PointCloud2 output;
        pcl::PCLPointCloud2Ptr cloud_tmp(new pcl::PCLPointCloud2());
        pcl::toPCLPointCloud2(*cloud, *cloud_tmp);
        pcl_conversions::fromPCL(*cloud_tmp, output);

        pub.publish(output);
        ros::spinOnce();
    }
    return 0;
}