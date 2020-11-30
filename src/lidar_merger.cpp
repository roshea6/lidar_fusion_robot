#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "laser_geometry/laser_geometry.h"
#include "std_msgs/Float32MultiArray.h"

sensor_msgs::LaserScanPtr pointcloud_to_laserscan(sensor_msgs::PointCloud2 *merged_cloud)
{
    sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
    output->header = merged_cloud->header;
    output->header.frame_id = "velodyne3";
    output->header.stamp = ros::Time::now();
    output->angle_min = -3.12414;
    // -179 degrees
    output->angle_max = 3.12414;
    // +179 degrees
    output->angle_increment = 0.0174532923847;
    output->time_increment = 0.000185185184819;
    output->scan_time = 0.0666666701436;
    output->range_min = .1;
    output->range_max = 100;
    float inf = std::numeric_limits<float>::infinity();
    uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
    output->ranges.assign(ranges_size, inf);
    output->intensities.assign(ranges_size, 0);
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*merged_cloud, "x"); it != it.end(); ++it)
    {
        const float &x = it[0];
        const float &y = it[1];
        const float &z = it[2];
        const float &intensity = it[3];
        if (std::isnan(x) || std::isnan(y) || std::isnan(z))
        {
            ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
            continue;
        }

        double range_sq = y * y + x * x;
        double intensity_sq = intensity;
        double range_min_sq_ = output->range_min * output->range_min;
        if (range_sq < range_min_sq_)
        {
            ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
            continue;
        }

        double angle = atan2(y, x);
        if (angle < output->angle_min || angle > output->angle_max)
        {
            ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
            continue;
        }
        int index = (angle - output->angle_min) / output->angle_increment;

        if (output->ranges[index] * output->ranges[index] > range_sq)
        {
            output->ranges[index] = sqrt(range_sq);
            output->intensities[index] = intensity_sq;
        }
    }

    return output;
}

int main(int argc, char **argv)
{
    sensor_msgs::PointCloud2 cloud_1;
    sensor_msgs::PointCloud2 cloud_2;
    sensor_msgs::PointCloud2 concatenated_cloud;

    sensor_msgs::LaserScan concatenated_scan;

    ros::init(argc, argv, "lidar_merger");

    ros::NodeHandle n;

    ros::Publisher merged_cloud_pub = n.advertise<sensor_msgs::PointCloud2>("concatenated_cloud", 1);
    ros::Publisher merged_scan_pub = n.advertise<sensor_msgs::LaserScan>("combined_scan", 1);

    while(ros::ok())
    {
        // Shared pointer to receive the point cloud message into
        boost::shared_ptr<sensor_msgs::PointCloud2 const> point_cloud_ptr;

        // Waits for 1 seconds for the next message from the first LiDAR
        point_cloud_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/velodyne_points1", ros::Duration(1));
        if (point_cloud_ptr == NULL)
            ROS_INFO("No laser messages received");
        else
            cloud_1 = *point_cloud_ptr;

        // Waits for 1 seconds from the next message from the second LiDAR
        point_cloud_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/velodyne_points2", ros::Duration(1));
        if (point_cloud_ptr == NULL)
            ROS_INFO("No laser messages received");
        else
            cloud_2 = *point_cloud_ptr;

        pcl::concatenatePointCloud(cloud_1, cloud_2, concatenated_cloud);
        concatenated_cloud.fields[3].name = "intensity";
        concatenated_cloud.header.frame_id = "velodyne3";

        merged_cloud_pub.publish(concatenated_cloud);

        concatenated_scan = *pointcloud_to_laserscan(&concatenated_cloud);
        merged_scan_pub.publish(concatenated_scan);
    }

    return 0;
}