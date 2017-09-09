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


struct Vec3
{
    float x;
    float y;
    float z;
};

float CylTest_CapsFirst( const Vec3 & pt1, const Vec3 & pt2, float lengthsq, float radius_sq, const Vec3 & testpt )
{
    float dx, dy, dz;	// vector d  from line segment point 1 to point 2
    float pdx, pdy, pdz;	// vector pd from point 1 to test point
    float dot, dsq;

    dx = pt2.x - pt1.x;	// translate so pt1 is origin.  Make vector from
    dy = pt2.y - pt1.y;     // pt1 to pt2.  Need for this is easily eliminated
    dz = pt2.z - pt1.z;

    pdx = testpt.x - pt1.x;		// vector from pt1 to test point.
    pdy = testpt.y - pt1.y;
    pdz = testpt.z - pt1.z;

    // Dot the d and pd vectors to see if point lies behind the
    // cylinder cap at pt1.x, pt1.y, pt1.z

    dot = pdx * dx + pdy * dy + pdz * dz;

    // If dot is less than zero the point is behind the pt1 cap.
    // If greater than the cylinder axis line segment length squared
    // then the point is outside the other end cap at pt2.

    if( dot < 0.0f || dot > lengthsq )
    {
        return( -1.0f );
    }
    else
    {
        // Point lies within the parallel caps, so find
        // distance squared from point to line, using the fact that sin^2 + cos^2 = 1
        // the dot = cos() * |d||pd|, and cross*cross = sin^2 * |d|^2 * |pd|^2
        // Carefull: '*' means mult for scalars and dotproduct for vectors
        // In short, where dist is pt distance to cyl axis:
        // dist = sin( pd to d ) * |pd|
        // distsq = dsq = (1 - cos^2( pd to d)) * |pd|^2
        // dsq = ( 1 - (pd * d)^2 / (|pd|^2 * |d|^2) ) * |pd|^2
        // dsq = pd * pd - dot * dot / lengthsq
        //  where lengthsq is d*d or |d|^2 that is passed into this function

        // distance squared to the cylinder axis:

        dsq = (pdx*pdx + pdy*pdy + pdz*pdz) - dot*dot/lengthsq;

        if( dsq > radius_sq )
        {
            return( -1.0f );
        }
        else
        {
            return( dsq );		// return distance squared to axis
        }
    }
}

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


int main(int argc, char **argv)
{
    pcl::PointCloud<PointT>::Ptr cloud_blob (new pcl::PointCloud<PointT>);

    // Fill in the cloud data
    pcl::PCDReader reader;
    reader.read ("/home/cosc/student/dlb74/catkin_ws/src/branch_detection2/src/cloud_filtered.pcd", *cloud_blob);

    int num_points = cloud_blob->size();

    pcl::PointCloud<PointT>::Ptr cloud_DownSampled (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    pcl::ModelCoefficients::Ptr coefficients_cylinder_trunk (new pcl::ModelCoefficients);
    coefficients_cylinder_trunk->values.resize (7);
    pcl::PointCloud<PointNT>::Ptr trunk_normals (new pcl::PointCloud<PointNT>);
    trunk_normals->points.resize(cloud_blob->size());
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

    while (!viewer->wasStopped ())
    {

        //cout << coefficients_cylinder_trunk->values[0] << "a" << coefficients_cylinder_trunk->values[1] << "b"
          //   << coefficients_cylinder_trunk->values[2] << "c" << coefficients_cylinder_trunk->values[3] << "d"
            // << coefficients_cylinder_trunk->values[4] << "e" << coefficients_cylinder_trunk->values[5] << "f"
             //<< coefficients_cylinder_trunk->values[6] << "g";
        // 2.67866a1b-0.981821c0.587474d0.0747359e0.805785f0.275014g

        viewer->removeAllShapes();
        viewer->updatePointCloud(cloud_filtered, "Filtered Cloud");
        viewer->addCylinder(*coefficients_cylinder_trunk, "sadfsaf");
        viewer->spinOnce(100);

        viewer2->removeAllShapes();
        viewer2->updatePointCloud(cloud_after_trunk_seg, "Filtered Cloud");
        viewer2->addCylinder(*coefficients_cylinder_trunk, "sadfs3af");
        viewer2->spinOnce(100);

    }

    return (0);

}