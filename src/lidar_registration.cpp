#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/point_cloud.h"
#include "pcl_ros/filters/statistical_outlier_removal.h"
#include "pcl_ros/filters/voxel_grid.h"
#include "pcl_ros/features/normal_3d.h"
#include "pcl/registration/icp_nl.h"
#include "pcl_ros/impl/transforms.hpp"
#include <string.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_registration");
    ros::NodeHandle nh;

    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef pcl::PointNormal PointNormalT;
    typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_matched",1);
    ros::Publisher cloud_A = nh.advertise<sensor_msgs::PointCloud2>("cloud_1",1);
    ros::Publisher cloud_B = nh.advertise<sensor_msgs::PointCloud2>("cloud_2",1);

    // Our two input point clouds and stitched ouput pointcloud to be published
    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 cloud_1_out;
    sensor_msgs::PointCloud2 cloud_2_out;

    // PCL point clouds which will be used for all the backend stitching
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt( new pcl::PointCloud<pcl::PointXYZ>() );
    PointCloud::Ptr src (new PointCloud);
    PointCloud::Ptr tgt (new PointCloud);

    // Reads in the PCL point clouds from files
    // pcl::io::loadPCDFile("/home/ryan/catkin_ws/src/lidar_fusion_robot/capture0002.pcd",*cloud_tgt);
    // pcl::io::loadPCDFile("/home/ryan/catkin_ws/src/lidar_fusion_robot/capture0003.pcd",*cloud_src);

    while(ros::ok())
    {
        // Shared pointer to receive the point cloud message into
        boost::shared_ptr<sensor_msgs::PointCloud2 const> point_cloud_ptr;

        // Read in data from the ROS pointcloud topics
        point_cloud_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/velodyne_points1", ros::Duration(1));
        if (point_cloud_ptr == NULL)
            ROS_INFO("No laser messages received");
        else
            cloud_1_out = *point_cloud_ptr;

        point_cloud_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/velodyne_points2", ros::Duration(1));
        if (point_cloud_ptr == NULL)
            ROS_INFO("No laser messages received");
        else
            cloud_2_out = *point_cloud_ptr;

        // Convert the ROS point clouds into PCL point clouds
        pcl::fromROSMsg(cloud_1_out, *cloud_tgt);
        pcl::fromROSMsg(cloud_2_out, *cloud_src);

        // Implements filtering & downsampling of Cloud 1
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
        statFilter.setInputCloud(cloud_src);
        statFilter.setMeanK(10);
        statFilter.setStddevMulThresh(0.2);
        statFilter.filter(*cloud_src);

        pcl::VoxelGrid<pcl::PointXYZ> voxelSampler; 
        voxelSampler.setInputCloud(cloud_src); 
        voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f); 
        voxelSampler.filter(*src); 

        // Implements filtering & downsampling of Cloud 2
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter2;
        statFilter2.setInputCloud(cloud_tgt);
        statFilter2.setMeanK(10);
        statFilter2.setStddevMulThresh(0.2);
        statFilter2.filter(*cloud_tgt);

        pcl::VoxelGrid<pcl::PointXYZ> voxelSampler2; 
        voxelSampler2.setInputCloud(cloud_tgt); 
        voxelSampler2.setLeafSize(0.01f, 0.01f, 0.01f); 
        voxelSampler2.filter(*tgt); 

        PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
        PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

        pcl::NormalEstimation<PointT, PointNormalT> norm_est;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        norm_est.setSearchMethod (tree);
        norm_est.setKSearch (30);

        norm_est.setInputCloud (src);
        norm_est.compute (*points_with_normals_src);
        pcl::copyPointCloud (*src, *points_with_normals_src);

        norm_est.setInputCloud (tgt);
        norm_est.compute (*points_with_normals_tgt);
        pcl::copyPointCloud (*tgt, *points_with_normals_tgt);

        // Implements ICP Algorithm for Registration 1 & 2
        pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> icp;
        icp.setInputSource(points_with_normals_src);
        icp.setInputTarget(points_with_normals_tgt);

        icp.setMaxCorrespondenceDistance(0.1);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon (1e-6);
        icp.setEuclideanFitnessEpsilon(0.1);

        Eigen::Matrix4f final_transform;
        Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;
        PointCloudWithNormals::Ptr reg_result = points_with_normals_src;

        for (int i = 0; i < 30; ++i)  
        {
            ROS_INFO ("Iteration Nr. %d.\n", i);

            // save cloud for visualization purpose
            points_with_normals_src = reg_result;

            // Estimate
            icp.setInputSource (points_with_normals_src);
            icp.align (*reg_result);

            //accumulate transformation between each Iteration
            Ti = icp.getFinalTransformation () * Ti;

            if (std::abs ((icp.getLastIncrementalTransformation () - prev).sum ()) < icp.getTransformationEpsilon ())
            icp.setMaxCorrespondenceDistance (icp.getMaxCorrespondenceDistance () - 0.001);

            prev = icp.getLastIncrementalTransformation ();
        }    
        // Get the transformation from target to source
        targetToSource = Ti.inverse();

        // Transform target back in source frame
        pcl::transformPointCloud (*cloud_tgt, *cloud_aligned, targetToSource);

        //add the source to the transformed target
        *cloud_aligned += *cloud_src;

        final_transform = targetToSource;

        pcl::toROSMsg(*cloud_aligned, output);
        output.header.frame_id = "velodyne3";

        // pcl::toROSMsg(*cloud_src, cloud_1_out);
        cloud_1_out.header.frame_id = "velodyne3";

        // pcl::toROSMsg(*cloud_tgt, cloud_2_out);
        cloud_2_out.header.frame_id = "velodyne3";

        pcl_pub.publish(output);
        cloud_A.publish(cloud_1_out);
        cloud_B.publish(cloud_2_out);
    }

    // while(ros::ok){
    //     pcl_pub.publish(output);
    //     cloud_A.publish(cloud_1_out);
    //     cloud_B.publish(cloud_2_out);
    //     ros::spinOnce();
    // }
    return 0;
}