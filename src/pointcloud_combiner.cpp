#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
// #include "sensor_msgs/point_cloud_conversion.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_assembler/AssembleScans2.h>
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/filters/statistical_outlier_removal.h"
#include "pcl_ros/filters/voxel_grid.h"
#include "pcl_ros/features/normal_3d.h"
#include "pcl/registration/icp_nl.h"
#include "pcl_ros/impl/transforms.hpp"
#include <string.h>

class cloudConverter
{
private:
    // Nodehandle for the class
    ros::NodeHandle n_;

    // Publisher for PointCloud2 assemled message
    ros::Publisher cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("assembled_cloud", 400);

    // Publisher for pushing the the two velodyne topics onto the same topic
    ros::Publisher combined_cloud_pub_ = n_.advertise<sensor_msgs::PointCloud2>("combined_cloud", 400);

    // Service client
    ros::ServiceClient client_ = n_.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");

    // AssembleScans2 service request
    laser_assembler::AssembleScans2 srv_;



public:
    cloudConverter() {}

    void assemblerCallback(const sensor_msgs::PointCloud2 cloud2)
    {
        // Publish the cloud onto the shared topic
        combined_cloud_pub_.publish(cloud2);

        // Set the desired time frame for the clouds to merge (Just beginning to now for now)
        srv_.request.begin = ros::Time(0);
        srv_.request.end = ros::Time::now();

        if (client_.call(srv_))
        {  
            // Get the message with the assembled pointcloud from the request reponse
            sensor_msgs::PointCloud2 returned_cloud = srv_.response.cloud;
            // std::cout << returned_cloud << std::endl;
            cloud_pub_.publish(returned_cloud);
        }
        else
        {
            printf("Service call failed\n");
        }
    }

    void secoundCloudCallback(const sensor_msgs::PointCloud2 cloud)
    {
        // Publish the cloud onto the shared topic
        combined_cloud_pub_.publish(cloud);
    }

};

int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "cloud_converter");

    // Define the nodehandle
    ros::NodeHandle n;

    // Wait for the service to be available
    ros::service::waitForService("assemble_scans");

    // Make a cloudConverter object
    cloudConverter converter;

    // Use the converters callback function for the subscriber
    ros::Subscriber cloud2_sub = n.subscribe("/velodyne_points1", 100, &cloudConverter::assemblerCallback, &converter);

    // Use the callback to push the second Velodyne's pointcloud onto the shared topic
    ros::Subscriber second_cloud2_sub = n.subscribe("/velodyne_points2", 100, &cloudConverter::secoundCloudCallback, &converter);
    
    ros::spin();
    
    return 0;

}