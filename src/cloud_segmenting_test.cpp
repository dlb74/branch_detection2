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

#include <string>
#include <boost/lexical_cast.hpp>

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
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>


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

#define BRANCH_NORM_KSEARCH_RADIUS 0.01
#define BRANCHSEG_NORMDIST_WEIGHT 0.01
#define BRANCHSEG_CYLDIST_THRESH 0.02
#define BRANCHSEG_CYLRAD_MIN 0.01
#define BRANCHSEG_CYLRAD_MAX 0.1


struct Vec3
{
    float x;
    float y;
    float z;
};

float Calculate_DotProduct(Vec3 a, Vec3 b) {
    float dx, dy, dz;
    dx = a.x * b.x;
    dy = a.y * b.y;
    dz = a.z * b.z;

    return (dx + dy + dz);
}

float Calculate_Magnitude(Vec3 vector) {
    return sqrt((vector.x * vector.x) + (vector.y * vector.y) + (vector.z * vector.z));
}

float CalculateAngle(Vec3 AB, Vec3 AC) {
    float AB_AC_dotProduct = Calculate_DotProduct(AB, AC);
    float magnitude_AB = Calculate_Magnitude(AB);
    float magnitude_AC = Calculate_Magnitude(AC);

    return acos(AB_AC_dotProduct/(magnitude_AB*magnitude_AC));
}

float CalculateDistanceFromAxis(Vec3 point_A, Vec3 line, Vec3 point_B) {
    Vec3 BA = {point_A.x - point_B.x, point_A.y - point_B.y, point_A.z - point_B.z};
    Vec3 BALineCross = {(BA.y * line.z) - (BA.z * line.y),
                        (BA.z * line.x) - (BA.x * line.z),
                        (BA.x * line.y) - (BA.y * line.x)};
    float cross_magnitude = Calculate_Magnitude(BALineCross);
    float line_magnitude = Calculate_Magnitude(line);
    return (cross_magnitude/line_magnitude);
}

float DistanceBetweenTwoPoints(Vec3 point_A, Vec3 point_B) {
    return sqrt(pow((point_A.x - point_B.x),2) + pow((point_A.x - point_B.x),2) + pow((point_A.x - point_B.x),2));
}

/**
 * Checks if a point lies in a cylinder
 * @param point_C the point to be checked
 * @param coefficients_cylinder the cylinder which may or may not contain the point
 * @return false if it is not in the cylinder, true if it is
 */
bool InsideCylinder(Vec3 point_C, pcl::ModelCoefficients::Ptr coefficients_cylinder_trunk) {
    Vec3 point_A = {coefficients_cylinder_trunk->values[0], coefficients_cylinder_trunk->values[1], coefficients_cylinder_trunk->values[2]};
    Vec3 point_B = {coefficients_cylinder_trunk->values[0] + coefficients_cylinder_trunk->values[3],
                   coefficients_cylinder_trunk->values[1] + coefficients_cylinder_trunk->values[4],
                   coefficients_cylinder_trunk->values[2] + coefficients_cylinder_trunk->values[5]};

    Vec3 AB = {point_B.x - point_A.x, point_B.y - point_A.y, point_B.z - point_A.z};
    Vec3 AC = {point_C.x - point_A.x, point_C.y - point_A.y, point_C.z - point_A.z};
    Vec3 BC = {point_C.x - point_B.x, point_C.y - point_B.y, point_C.z - point_B.z};
    Vec3 BA = {point_A.x - point_B.x, point_A.y - point_B.y, point_A.z - point_B.z};

    float AB_AC_angle = CalculateAngle(AB, AC);
    float BA_BC_angle = CalculateAngle(BA, BC);

    if (AB_AC_angle > 3.1415 || AB_AC_angle < 0 || BA_BC_angle > 3.1415 || BA_BC_angle < 0) {
        return false;
    }

    float distanceFromLine = CalculateDistanceFromAxis(point_A,  AB,  point_C);

    if (distanceFromLine > coefficients_cylinder_trunk->values[6]) {
        return false;
    }else {
        return true;
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

    coefficients_cylinder->values[0] -= (coefficients_cylinder->values[3] * .5);
    coefficients_cylinder->values[1] -= (coefficients_cylinder->values[4] * .5);
    coefficients_cylinder->values[2] -= (coefficients_cylinder->values[5] * .5);

    coefficients_cylinder->values[3] *= 4;
    coefficients_cylinder->values[4] *= 4;
    coefficients_cylinder->values[5] *= 4;
    coefficients_cylinder->values[6] *= 1.8;
}

void Setup_Second_Cylinder(pcl::ModelCoefficients::Ptr original_cylinder, pcl::ModelCoefficients::Ptr second_cylinder) {

    second_cylinder->values[0] = original_cylinder->values[0];
    second_cylinder->values[1] = original_cylinder->values[1];
    second_cylinder->values[2] = original_cylinder->values[2];
    second_cylinder->values[3] = original_cylinder->values[3];
    second_cylinder->values[4] = original_cylinder->values[4];
    second_cylinder->values[5] = original_cylinder->values[5];
    second_cylinder->values[6] = original_cylinder->values[6] * 1.5;
}

void Cylinder_Seg( pcl::PointCloud<PointT>::Ptr cloud,
                   pcl::PointCloud<PointNT>::Ptr cloud_normals,
                   pcl::ModelCoefficients::Ptr coefficients_cylinder,
                   pcl::ModelCoefficients::Ptr coefficients_cylinder_original,
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

    coefficients_cylinder_original->values = coefficients_cylinder->values;
    Lengthen_Cylinder(coefficients_cylinder);

    // Extract cylinder inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers_trunk);
    extract.setNegative (true);
    extract.filter (*cloud_remainder);
}

bool customRegionGrowing (const pcl::PointXYZ& point_a, const pcl::PointXYZ& point_b, float squared_distance)
{
    Vec3 vec_a = {point_a.x, point_a.y, point_a.z};
    Vec3 vec_b = {point_b.x, point_b.y, point_b.z};

    float distance = DistanceBetweenTwoPoints(vec_a, vec_b);

    if (DistanceBetweenTwoPoints(vec_a, vec_b) > 0.005) { //0.005 gives 10 clusters
        return false;
    } else {
        return true;
    }
}

/**
 * From the Conditional Euclidean Clustering tutorial
 * @param final_pointcloud the pointcloud to be processed
 * @param clusters the object where the clusters are stored
 */
void segmentUsingCorrespondenceGrouping(pcl::PointCloud<PointT>::Ptr final_pointcloud, pcl::IndicesClustersPtr clusters) {

    pcl::PointCloud<PointT>::Ptr cluster_pointcloud1 (new pcl::PointCloud<PointT>);
    std::vector<pcl::PointXYZ> group1;

    pcl::ConditionalEuclideanClustering<PointT> cec (true);
    cec.setInputCloud (final_pointcloud);
    cec.setConditionFunction (&customRegionGrowing);
    cec.setClusterTolerance (500.0);
    cec.setMinClusterSize (final_pointcloud->points.size () / 50);
    cec.setMaxClusterSize (final_pointcloud->points.size ());
    cec.segment (*clusters);

    for (int i = 0; i < clusters->size(); ++i)
    {
        ROS_WARN("Cluster size: %lu ",  (*clusters)[i].indices.size ());

        for (int j = 0; j < (*clusters)[i].indices.size (); ++j) {
            //group1.push_back(final_pointcloud->points[(*clusters)[i].indices[j]]);
        }
    }
}

void regionGrowingSegmentation(pcl::PointCloud<PointT>::Ptr cloud) {

    ROS_WARN("%lu", cloud->points.size());

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (5);
    reg.setMaxClusterSize (5000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (15);
    reg.setInputCloud (cloud);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (200.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (50.00);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud(); //This isn't working properly
    ROS_WARN("%lu", colored_cloud->points.size());

    pcl::visualization::CloudViewer viewer ("Cluster viewer");
    viewer.showCloud(colored_cloud);

    while (!viewer.wasStopped ())
    {

    }
}

void findBranches(pcl::PointCloud<PointT>::Ptr input_pointcloud, std::vector<pcl::ModelCoefficients::Ptr>* coefficients_vector) {

    pcl::PointCloud<PointNT>::Ptr branch_normals (new pcl::PointCloud<PointNT>);
    branch_normals->points.resize(input_pointcloud->size());

    pcl::ModelCoefficients::Ptr new_coefficients (new pcl::ModelCoefficients);
    new_coefficients->values.resize (7);

    pcl::ModelCoefficients::Ptr spare_coefficients (new pcl::ModelCoefficients);
    spare_coefficients->values.resize (7);

    pcl::PointCloud<PointT>::Ptr cloud_after (new pcl::PointCloud<PointT>);

    Norm_Est( input_pointcloud, branch_normals, BRANCH_NORM_KSEARCH_RADIUS );
    branch_normals->width = (int)branch_normals->points.size();

    Cylinder_Seg( input_pointcloud, branch_normals,
                  new_coefficients, spare_coefficients, cloud_after, BRANCHSEG_NORMDIST_WEIGHT,
                  BRANCHSEG_CYLDIST_THRESH, BRANCHSEG_CYLRAD_MIN, BRANCHSEG_CYLRAD_MAX, pcl::SACMODEL_LINE);

    coefficients_vector->push_back(new_coefficients);


    input_pointcloud->points = cloud_after->points;

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
    pcl::ModelCoefficients::Ptr coefficients_cylinder_trunk_original (new pcl::ModelCoefficients);
    coefficients_cylinder_trunk->values.resize (7);
    pcl::PointCloud<PointNT>::Ptr trunk_normals (new pcl::PointCloud<PointNT>);
    trunk_normals->points.resize(cloud_blob->size());
    pcl::PointCloud<PointT>::Ptr cloud_after_trunk_seg (new pcl::PointCloud<PointT> ());

    DownSample(cloud_blob, cloud_DownSampled );
    cloud_DownSampled->width = (int)cloud_DownSampled->points.size();

    Frame_Filter( cloud_DownSampled, cloud_filtered );
    cloud_filtered->width = (int)cloud_filtered->points.size();

    Norm_Est( cloud_filtered, trunk_normals, TRUNK_NORM_KSEARCH_RADIUS );
    trunk_normals->width = (int)trunk_normals->points.size();

    Cylinder_Seg( cloud_filtered, trunk_normals,
                  coefficients_cylinder_trunk, coefficients_cylinder_trunk_original, cloud_after_trunk_seg, TRUNKSEG_NORMDIST_WEIGHT,
                  TRUNKSEG_CYLDIST_THRESH, TRUNKSEG_CYLRAD_MIN, TRUNKSEG_CYLRAD_MAX, pcl::SACMODEL_CYLINDER);

    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > cloud_points = cloud_filtered->points;

    for (int i = 0; i < cloud_points.size(); i++) {

        Vec3 cloudPoint = {cloud_points[i].x, cloud_points[i].y, cloud_points[i].z};
        if (InsideCylinder(cloudPoint, coefficients_cylinder_trunk)) {
            cloud_points.erase(cloud_points.begin() + i);
            i--;
        }
    }

    pcl::ModelCoefficients::Ptr coefficients_second_cylinder (new pcl::ModelCoefficients);
    coefficients_second_cylinder->values.resize (7);
    Setup_Second_Cylinder(coefficients_cylinder_trunk, coefficients_second_cylinder);

    for (int i = 0; i < cloud_points.size(); i++) {

        Vec3 cloudPoint = {cloud_points[i].x, cloud_points[i].y, cloud_points[i].z};
        if (!InsideCylinder(cloudPoint, coefficients_second_cylinder)) {
            cloud_points.erase(cloud_points.begin() + i);
            i--;

        }
    }

    pcl::PointCloud<PointT>::Ptr segmented_pointcloud (new pcl::PointCloud<PointT>);
    segmented_pointcloud->points = cloud_points;
    ROS_WARN("segmented pointcloud %lu", segmented_pointcloud->size());

    pcl::PointCloud<PointT>::Ptr final_pointcloud (new pcl::PointCloud<PointT>);
    final_pointcloud->points = cloud_points;

    Frame_Filter(segmented_pointcloud, final_pointcloud );
    final_pointcloud->width = (int)final_pointcloud->points.size();
    ROS_WARN("final pointcloud %lu", final_pointcloud->size());

    std::vector<pcl::ModelCoefficients::Ptr> coefficients_vector;
/*
    for (int i = 0; i < 10;i++) {
        findBranches(final_pointcloud, &coefficients_vector);
        ROS_WARN("coefficients vector size %lu", coefficients_vector.size());
    }
*/
    pcl::IndicesClustersPtr clusters (new pcl::IndicesClusters);
    segmentUsingCorrespondenceGrouping(final_pointcloud, clusters);
    ROS_WARN("Number of clusters: %lu \n", clusters->size());
    //regionGrowingSegmentation(final_pointcloud);

    pcl::PointCloud <PointT>::Ptr fouth_nuageobjets(new pcl::PointCloud <PointT>);

    /*
    pcl::ExtractIndices<PointT> filtrerG (true);
    filtrerG.setInputCloud(final_pointcloud);
    filtrerG.setIndices((*clusters)[3].indices); //indices of the fouth cluster.
    filtrerG.filter(*fouth_nuageobjets);
*/
/*
    for (int it = clusters.begin (); it != clusters.end (); ++it)
    {
        fouth_nuageobjets->clear();
        for (int pit = 0; pit < (*clusters)[3].indices.size(); pit++) {
            fouth_nuageobjets->points.push_back(final_pointcloud->points[*pit]); //*
        }
        fouth_nuageobjets->width = fouth_nuageobjets->points.size ();
        //fouth_nuageobjets->height = 1;
        //fouth_nuageobjets->is_dense = true;
    }*/

    //ROS_WARN("%lu", clusters.size());
//    ROS_WARN("Hi %lu %d\n",  cloud_points.size(), si2ze);





    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer6 (new pcl::visualization::PCLVisualizer ("3D Viewer")); //vizualiser
    viewer6->initCameraParameters( );
    viewer6->setShowFPS( false );
    viewer6->setBackgroundColor (0, 0, 0);
    viewer6->addPointCloud<PointT> (final_pointcloud, "Filtered Cloud");

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer5 (new pcl::visualization::PCLVisualizer ("3D Viewer")); //vizualiser
    viewer5->initCameraParameters( );
    viewer5->setShowFPS( false );
    viewer5->setBackgroundColor (0, 0, 0);
    viewer5->addPointCloud<PointT> (cloud_filtered, "Filtered Cloud");

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer4 (new pcl::visualization::PCLVisualizer ("3D Viewer")); //vizualiser
    viewer4->initCameraParameters( );
    viewer4->setShowFPS( false );
    viewer4->setBackgroundColor (0, 0, 0);
    viewer4->addPointCloud<PointT> (cloud_filtered, "Filtered Cloud");

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer3 (new pcl::visualization::PCLVisualizer ("3D Viewer")); //vizualiser
    viewer3->initCameraParameters( );
    viewer3->setShowFPS( false );
    viewer3->setBackgroundColor (0, 0, 0);
    viewer3->addPointCloud<PointT> (cloud_filtered, "Filtered Cloud");

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer")); //vizualiser
    viewer2->initCameraParameters( );
    viewer2->setShowFPS( false );
    viewer2->setBackgroundColor (0, 0, 0);
    viewer2->addPointCloud<PointT> (cloud_filtered, "Filtered Cloud");

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer")); //vizualiser
    viewer->initCameraParameters( );
    viewer->setShowFPS( false );
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<PointT> (cloud_blob, "Filtered Cloud");

    while (!viewer->wasStopped ())
    {
        viewer6->removeAllShapes();
        viewer6->updatePointCloud(final_pointcloud, "Filtered Cloud6");

        viewer5->removeAllShapes();
        viewer5->updatePointCloud(cloud_filtered, "Filtered Cloud5");
        viewer5->addCylinder(*coefficients_cylinder_trunk, "inner_cylinder5");
        viewer5->addCylinder(*coefficients_second_cylinder, "second_cylinder5");

        viewer4->removeAllShapes();
        viewer4->updatePointCloud(cloud_filtered, "Filtered Cloud4");
        viewer4->addCylinder(*coefficients_cylinder_trunk, "inner_cylinder4");

        viewer3->removeAllShapes();
        viewer3->updatePointCloud(cloud_filtered, "Filtered Cloud3");
        viewer3->addCylinder(*coefficients_cylinder_trunk_original, "inner_cylinder3");

        viewer2->removeAllShapes();
        viewer2->updatePointCloud(cloud_filtered, "Filtered Cloud2");

        viewer->removeAllShapes();
        viewer->updatePointCloud(cloud_blob, "Filtered Cloud");
        viewer->spinOnce(100);


    }

    return (0);
}

/*
        for (int i = 0 ; i < coefficients_vector.size(); i++) {
            viewer->addCylinder(*coefficients_vector[i], "inner_cylinder" + boost::lexical_cast<string>(i));
        }
        viewer->spinOnce(100);
*/