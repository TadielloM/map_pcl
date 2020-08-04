#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
tf::TransformListener* listener;

void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // ROS_INFO("Point Cloud Map: callback");
    //Converting ROSmsg to pPCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*msg, *tmp_cloud);
    //Filtering input cloud
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (tmp_cloud);
    sor.setLeafSize (0.5, 0.5, 0.5); //50cm filter
    sor.filter (*tmp_cloud);
    //Add new input to previus cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

    pcl_ros::transformPointCloud("map", *tmp_cloud, *tmp_cloud2, *listener);
    *cloud = *cloud + *tmp_cloud2;
}

bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  cloud->clear();
  // ROS_INFO("Point Cloud Map Resetting");
  return true;
}

int main(int argc, char** argv){
    //Initialize ROS
    ros::init (argc,argv,"map_pcl");
    ros::NodeHandle nh;
    ros::Duration(5).sleep();
    listener = new(tf::TransformListener);
    //Set up subscriber and Publisher
    ros::Subscriber velodyne_subscriber;
    velodyne_subscriber = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, velodyne_callback);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("/map_points",1);
    ros::ServiceServer resetService = nh.advertiseService("/map_pcl/reset", resetSrv);
    //Setting rate of one second
    ros::Rate rate(1);

    while(ros::ok()){
        //listen tf transform
        tf::StampedTransform transform;
        try{
          listener->lookupTransform("/velodyne_base", "/map",  
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
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

        output.header.frame_id = "map";

        pub.publish(output);
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}