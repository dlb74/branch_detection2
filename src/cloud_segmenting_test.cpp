#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include "ros/ros.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include <sstream>
#include <sensor_msgs/PointCloud2.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <vector>
#include <Eigen/Dense>
#include <pcl/ModelCoefficients.h>

#include <iostream>
#include <fstream>
#include <ctime>
#include <math.h>
#include <string>
#include <iostream>
#include <iostream>


using namespace std;


typedef pcl::PointXYZ PointT;
typedef pcl::Normal PointNT;

#define DWNSMPL_SQRLEAF_SIZE 0.02

#define KMEANFILTER_RANGE 150
#define KMEANFILTER_THRESH_STDVMUL 0.8

#define TRUNK_NORM_KSEARCH_RADIUS 0.05
#define TRUNKSEG_NORMDIST_WEIGHT 0.05
#define TRUNKSEG_CYLDIST_THRESH 0.01
#define TRUNKSEG_CYLRAD_MIN 0.09
#define TRUNKSEG_CYLRAD_MAX 1


/** PCL Frame Normal Estimation */
void Norm_Est( pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointNT>::Ptr cloud_normals, float normKSearchRadius )
{
    pcl::NormalEstimation<PointT, PointNT> ne;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    // Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud);
    ne.setRadiusSearch ( normKSearchRadius );
    ne.compute (*cloud_normals);
}


/** Downsampling the point cloud */
void DownSample( pcl::PointCloud<PointT>::Ptr cloud,
                 pcl::PointCloud<PointT>::Ptr cloud_DownSampled )
{
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize ( DWNSMPL_SQRLEAF_SIZE, DWNSMPL_SQRLEAF_SIZE, DWNSMPL_SQRLEAF_SIZE );
    sor.filter (*cloud_DownSampled);
}


/** PCL Frame Filtering */
void Frame_Filter( pcl::PointCloud<PointT>::Ptr cloud,
                   pcl::PointCloud<PointT>::Ptr cloud_filtered )
{
    // K-mean Statistical filtering
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (KMEANFILTER_RANGE);
    sor.setStddevMulThresh (KMEANFILTER_THRESH_STDVMUL);
    sor.filter (*cloud_filtered);
}

void Lengthen_Cylinder(pcl::ModelCoefficients::Ptr coefficients_cylinder) {

    coefficients_cylinder->values[3] *= 2;//x2  + (2 * x_delta);
    coefficients_cylinder->values[4] *= 2;//y1 + (2 * y_delta);
    coefficients_cylinder->values[5] *= 2;//z1 + (2 * z_delta);

    coefficients_cylinder->values[6] *= 1.5;
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

    Lengthen_Cylinder(coefficients_cylinder);

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
    //pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    //reader.read ("cloud_filtered.pcd", *cloud_blob);
    reader.read ("/home/cosc/student/dlb74/catkin_ws/src/branch_detection2/src/cloud_filtered.pcd", *cloud_blob);

    int num_points = cloud_blob->size();

    pcl::PointCloud<PointT>::Ptr cloud_DownSampled (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    pcl::ModelCoefficients::Ptr coefficients_cylinder_trunk (new pcl::ModelCoefficients);
    coefficients_cylinder_trunk->values.resize (7);
    pcl::PointCloud<PointNT>::Ptr trunk_normals (new pcl::PointCloud<PointNT>);
    trunk_normals->points.resize(1496064);
    pcl::PointCloud<PointT>::Ptr cloud_after_trunk_seg (new pcl::PointCloud<PointT> ());

    string str = "";

    DownSample( cloud_blob, cloud_DownSampled );
    cloud_DownSampled->width = (int)cloud_DownSampled->points.size();

    Frame_Filter( cloud_DownSampled, cloud_filtered );
    cloud_filtered->width = (int)cloud_filtered->points.size();

    Norm_Est( cloud_filtered, trunk_normals, TRUNK_NORM_KSEARCH_RADIUS );
    trunk_normals->width = (int)trunk_normals->points.size();

    Cylinder_Seg( cloud_filtered, trunk_normals,
                  coefficients_cylinder_trunk, cloud_after_trunk_seg, TRUNKSEG_NORMDIST_WEIGHT,
                  TRUNKSEG_CYLDIST_THRESH, TRUNKSEG_CYLRAD_MIN, TRUNKSEG_CYLRAD_MAX, pcl::SACMODEL_CYLINDER);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); //vizualiser
    viewer->initCameraParameters( );
    viewer->setShowFPS( false );
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (cloud_filtered, "Filtered Cloud");

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer")); //vizualiser
    viewer2->initCameraParameters( );
    viewer2->setShowFPS( false );
    viewer2->setBackgroundColor (0, 0, 0);
    viewer2->addPointCloud<PointT> (cloud_after_trunk_seg, "Filtered Cloud");

    //cout << coefficients_cylinder_trunk->values[1] << "t" << coefficients_cylinder_trunk->values[4] << "s";
    //0.195297t0.0747359s

//1,0,3
    //coefficients_cylinder_trunk->values[3] = -1.503712;
    //coefficients_cylinder_trunk->values[4] = -0.8505282;
    //coefficients_cylinder_trunk->values[5] = 2.593391;
    //coefficients_cylinder_trunk->values[6] = 0.4;


    while (!viewer->wasStopped ())
    {

        //cout << coefficients_cylinder_trunk->values[0] << "a" << coefficients_cylinder_trunk->values[1] << "b"
          //   << coefficients_cylinder_trunk->values[2] << "c" << coefficients_cylinder_trunk->values[3] << "d"
            // << coefficients_cylinder_trunk->values[4] << "e" << coefficients_cylinder_trunk->values[5] << "f"
             //<< coefficients_cylinder_trunk->values[6] << "g";
        // 2.67866a1b-0.981821c0.587474d0.0747359e0.805785f0.275014g

        //2.67866a1b-0.981821c0.587474d0.0747359e0.805785f0.275014g
        //A = (2.67866, 1, -0.981821)   B = (0.587474, 0.0747359, 0.805785)

        //AB = (-2.091186, -0.9252641, 1.787606)

        //A - 2AB = (-1.503712, -0.8505282, 2.593391)


        viewer->removeAllShapes();
        viewer->updatePointCloud(cloud_filtered, "Filtered Cloud");
        viewer->addCylinder(*coefficients_cylinder_trunk, "sadfsaf");
        viewer->spinOnce(100);

        viewer2->removeAllShapes();
        viewer2->updatePointCloud(cloud_after_trunk_seg, "Filtered Cloud");
        viewer2->addCylinder(*coefficients_cylinder_trunk, "sadfs3af");
        viewer2->spinOnce(100);

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