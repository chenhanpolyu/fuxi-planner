#include <ros/ros.h>
// PCL specific includes
#include <pcl/PCLPointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//添加引用

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include "pcl_ros/point_cloud.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

// 定义点云类型
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloudL;
typedef pcl::PointCloud<pcl::PointXYZ>  Cloud;
typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<pcl::Normal> Normal;

ros::Publisher pub;




void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //read raw point cloud data
    //and convert to raw_cloud
    sensor_msgs::PointCloud2 output;
    PointCloud::Ptr raw_cloud  (new PointCloud);
    output = *cloud_msg;
    pcl::fromROSMsg(output, *raw_cloud);   //

    //create temp point cloud ptr
    PointCloud::Ptr    cloud_filtered   (new PointCloud);
    PointCloud::Ptr    voxel_filtered   (new PointCloud);
    PointCloud::Ptr    r_filtered   (new PointCloud);


    // Perform the actual filtering-1
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (raw_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 4);
    pass.filter (*cloud_filtered);


    // Perform the actual filtering-2
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloud_filtered);
    sor.setLeafSize (0.17, 0.17, 0.2);
    sor.filter (*voxel_filtered);


    // Perform the actual filtering-3
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> outrem;
    outrem.setInputCloud(voxel_filtered);
    outrem.setRadiusSearch(0.35);
    outrem.setMinNeighborsInRadius (13);
    outrem.filter (*r_filtered);




    // Convert to ROS data type
//    sensor_msgs::PointCloud2 cloud_pt;
//    pcl_conversions::moveFromPCL(r_filtered, cloud_pt);
//      pcl_conversions::toROSMsg(r_filtered, cloud_pt)
//    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    // Publish the data
    pub.publish (r_filtered);

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "chen_filter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud 输入
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud 输出
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/filtered_RadiusOutlierRemoval", 1);

  // Spin
  ros::spin ();
}
