#include "ros/ros.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <math.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>


typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNT;


void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
}

void Cylinder_Seg( pcl::PointCloud<PointT>::Ptr cloud,
                   pcl::PointCloud<PointNT>::Ptr cloud_normals,
                   pcl::ModelCoefficients::Ptr coefficients_cylinder,
                   pcl::PointCloud<PointT>::Ptr cloud_remainder,
                   double normalWeight, double distanceThreshold, double radiusMinimum, double radiusMaximum,
                   int modelType)
{
    pcl::PointIndices::Ptr inliers_trunk (new pcl::PointIndices);
    pcl::SACSegmentationFromNormals<PointT, PointNT> seg;
    pcl::ExtractIndices<PointT> extract;

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (modelType);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight (normalWeight);
    seg.setMaxIterations (1000);
    seg.setDistanceThreshold (distanceThreshold);
    seg.setRadiusLimits (radiusMinimum, radiusMaximum);
    seg.setInputCloud (cloud);
    seg.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg.segment (*inliers_trunk, *coefficients_cylinder);

    // Extract cylinder inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_trunk);
    extract.setNegative (true);
    extract.filter (*cloud_remainder);
}

void Generate_Pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // Fill in the cloud data
    cloud->width  = 150;
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);

    // Generate the data
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        cloud->points[i].x = -100 + (rand () % 200);
        cloud->points[i].y = -100 + (rand () % 200);
        cloud->points[i].z = rand () % 50;
    }

    // Set a few outliers
    cloud->points[0].x = -120.0;
    cloud->points[3].x = -120.0;
    cloud->points[6].x = -120.0;
    cloud->points[0].z = -115.0;
    cloud->points[3].z = -125.0;
    cloud->points[6].z = -135.0;
}



int main(int argc, char **argv)
{

    pcl::PointCloud<PointT>::Ptr cloud_blob (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    //reader.read ("cloud_filtered.pcd", *cloud_blob);
    reader.read ("/home/cosc/student/dlb74/catkin_ws/src/branch_detection2/src/cloud_filtered.pcd", *cloud_blob);


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); //vizualiser
    viewer->initCameraParameters( );
    viewer->setShowFPS( false );
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (cloud_blob, "cloud_blob");

    while (!viewer->wasStopped ())
    {

        viewer->spinOnce(100);
    }
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //Generate_Pointcloud(cloud);

    /*
    Cylinder_Seg( pcl::PointCloud<PointT>::Ptr cloud,
            pcl::PointCloud<PointNT>::Ptr cloud_normals,
            pcl::ModelCoefficients::Ptr coefficients_cylinder,
            pcl::PointCloud<PointT>::Ptr cloud_remainder,
            double normalWeight, double distanceThreshold, double radiusMinimum, double radiusMaximum,
            int modelType)
*/





    return (0);

}