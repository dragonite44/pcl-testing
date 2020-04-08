/**********************************************************************
* Simple XYZ point cloud 3D visualiser of a .pcd file using PCL libraries
* The algorithm requires a .pcd file passed 
*
* The MIT License (MIT)
* Copyright (c) 2019 Felip Martí
* Swinburne University of Technology
* https://felipmarti.github.io
*
**********************************************************************/

 
#include <iostream>
#include <cstdlib>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>

int statisticalFilter(int argc, char** argv);
int zFilter(int argc, char** argv);
int combinedFilters(int argc, char** argv);
pcl::PointCloud<pcl::PointXYZ>::Ptr runBothFilters(int argc, char** argv);

/*
 *  Main
 **/

int main (int argc, char** argv)
{
	// pcl::PointCloud<pcl::PointXYZ>::Ptr filteredPoints=runBothFilters(argc, argv);

	// pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	// pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

	// const double distanceThreshold = std::strtod(argv[2],NULL);

	// pcl::SACSegmentation<pcl::PointXYZ> seg;
	// // seg.setOptimizeCoefficients(true);
	// seg.setModelType(pcl::SACMODEL_CIRCLE2D);
	// seg.setMethodType(pcl::SAC_RANSAC);
	// seg.setDistanceThreshold(distanceThreshold);

	// seg.setInputCloud(filteredPoints);
	// seg.segment(*inliers, *coefficients);

	// if (inliers->indices.size () == 0)
	// {
	// 	PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	// 	return (-1);
	// }

	// std::cout << "Model coefficients: " << coefficients->values[0] << " " 
	// 									<< coefficients->values[1] << " "
	// 									<< coefficients->values[2] << " " 
	// 									<< coefficients->values[3] << std::endl;

	// std::cout << "Model inliers: " << inliers->indices.size () << std::endl;

	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);

	// // for(size_t i=0;i<inliers->indices.size();i++)
	// // {
	// // 	cloudPtr->points.push_back(filteredPoints->points[inliers->indices[i]]);
	// // }

	// pcl::ModelCoefficients::Ptr projectionCoefficients(new pcl::ModelCoefficients());
	// projectionCoefficients->values.resize(4);
	// projectionCoefficients->values[0] = projectionCoefficients->values[1] = projectionCoefficients->values[3] = 0;
	// projectionCoefficients->values[2] = 1.0;

	// pcl::ProjectInliers<pcl::PointXYZ> proj;
	// proj.setModelType(pcl::SACMODEL_PLANE);
	// proj.setInputCloud(filteredPoints);
	// proj.setModelCoefficients(projectionCoefficients);
	// proj.filter(*cloudPtr);


 //    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("CLOUD"));   // Visualiser with the file name
 //    viewer->setBackgroundColor (0, 0, 0);                   // RGB 000->Black
 //    viewer->addPointCloud<pcl::PointXYZ> (filteredPoints, "OLDCLOUD");  // Cloud pointer and the name of the cloud
 //    // viewer->addPointCloud<pcl::PointXYZ> (cloudPtr, "CLOUD");  // Cloud pointer and the name of the cloud
 //    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "CLOUD");
 //    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "CLOUD");
 //    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "OLDCLOUD");
 //    viewer->initCameraParameters ();
 //    viewer->setCameraPosition(0, 0, -500,    0, 0, 0,   0, 0, 0,  0);
 //    // viewer->addCoordinateSystem(500.0);

 //    // Wait so the user can contemplate the amazing cloud
 //    while (!viewer->wasStopped ()) {
 //        viewer->spinOnce (100);
 //    }

	// return 0;
	// // return combinedFilters(argc, argv);

	return zFilter(argc,argv);
}

//Similar to combinedFilters, but it returns the filtered point cloud instead of visualising it
pcl::PointCloud<pcl::PointXYZ>::Ptr runBothFilters(int argc, char** argv)
{
	const double ZMIN=300;//these numbers looked good from testing
	const double ZMAX=4000;
	const double meanK=50;
	const double stddevMul=0.5;

    if (argc < 2) {
        printf("Usage: %s FILE.pcd\n",argv[0]);
        exit(1);
    }

    const char* file = argv[1];

    // Reading file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloudPtr) == -1) // Load the file into the cloud pointer
    {
        PCL_ERROR ("Couldn't read file: %s \n",file);
        exit(1);
    }

    std::cout << "Number of points before filtering: " << cloudPtr->points.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr zFilteredCloud (new pcl::PointCloud<pcl::PointXYZ>);

    //pass through filter
    pcl::PassThrough<pcl::PointXYZ> passthroughFilter;
    passthroughFilter.setInputCloud(cloudPtr);
    passthroughFilter.setFilterFieldName("z");
    passthroughFilter.setFilterLimits(ZMIN,ZMAX);
    passthroughFilter.filter(*zFilteredCloud);

    std::cout << "Number of points after Passthrough filtering: " << zFilteredCloud->points.size() << std::endl;

    //do statistical outlier filtering
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlierFilter;
    // outlierFilter.setInputCloud(zFilteredCloud);
    // outlierFilter.setMeanK(meanK);
    // outlierFilter.setStddevMulThresh(stddevMul);
    // outlierFilter.filter(*newCloud);

    // std::cout << "Number of points after StatOutlier filtering: " << newCloud->points.size() << std::endl;

    return zFilteredCloud;
}

//Demonstrates using both StatisticalOutlierRemoval and Passthrough filtering
int combinedFilters(int argc, char** argv)
{
	const double ZMIN=800;//these numbers looked good from testing
	const double ZMAX=3000;

    // Check input
    if (argc != 4) {
        printf("Usage: %s FILE.pcd meanK stddevMul\n",argv[0]);
        exit(1);
    }
    const char* file = argv[1];
    const double meanK = std::strtod(argv[2],NULL);
    const double stddevMul = std::strtod(argv[3],NULL);

    // Reading file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloudPtr) == -1) // Load the file into the cloud pointer
    {
        PCL_ERROR ("Couldn't read file: %s \n",file);
        return (-1);
    }

    std::cout << "Number of points before filtering: " << cloudPtr->points.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr zFilteredCloud (new pcl::PointCloud<pcl::PointXYZ>);

    //pass through filter
    pcl::PassThrough<pcl::PointXYZ> passthroughFilter;
    passthroughFilter.setInputCloud(cloudPtr);
    passthroughFilter.setFilterFieldName("z");
    passthroughFilter.setFilterLimits(ZMIN,ZMAX);
    passthroughFilter.filter(*zFilteredCloud);

    std::cout << "Number of points after Passthrough filtering: " << zFilteredCloud->points.size() << std::endl;

    //do statistical outlier filtering
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlierFilter;
    outlierFilter.setInputCloud(zFilteredCloud);
    outlierFilter.setMeanK(meanK);
    outlierFilter.setStddevMulThresh(stddevMul);
    outlierFilter.filter(*newCloud);

    std::cout << "Number of points after StatOutlier filtering: " << newCloud->points.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr removedPoints (new pcl::PointCloud<pcl::PointXYZ>);
    outlierFilter.setNegative(true);
    outlierFilter.filter(*removedPoints);

    const char* removedCloudName = "removedPoints";

    // Visualising cloud
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (file));   // Visualiser with the file name
    viewer->setBackgroundColor (0, 0, 0);                   // RGB 000->Black
    viewer->addPointCloud<pcl::PointXYZ> (newCloud, file);  // Cloud pointer and the name of the cloud
    viewer->addPointCloud<pcl::PointXYZ> (removedPoints, removedCloudName);  // Cloud pointer and the name of the cloud
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, file);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, removedCloudName);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, removedCloudName); //draw the removed points in red
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0, 0, 8000,    0, 0, 0,   0, 0, 0,  0);
    // viewer->addCoordinateSystem(500.0);

    // Wait so the user can contemplate the amazing cloud
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
    }

}

//Function to demonstrate StatisticalOutlierRemoval filtering, and drawing two point clouds in the same visualiser
int statisticalFilter(int argc, char** argv)
{
    // Check input
    if (argc != 4) {
        printf("Usage: %s FILE.pcd meanK stddevMul\n",argv[0]);
        exit(1);
    }
    const char* file = argv[1];
    const double meanK = std::strtod(argv[2],NULL);
    const double stddevMul = std::strtod(argv[3],NULL);

    // Reading file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloudPtr) == -1) // Load the file into the cloud pointer
    {
        PCL_ERROR ("Couldn't read file: %s \n",file);
        return (-1);
    }

    std::cout << "Number of points before filtering: " << cloudPtr->points.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZ>);

    //do statistical outlier filtering
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(cloudPtr);
    filter.setMeanK(meanK);
    filter.setStddevMulThresh(stddevMul);
    filter.filter(*newCloud);

    std::cout << "Number of points after filtering: " << newCloud->points.size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr removedPoints (new pcl::PointCloud<pcl::PointXYZ>);
    filter.setNegative(true);
    filter.filter(*removedPoints);

    const char* removedCloudName = "removedPoints";

    // Visualising cloud
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (file));   // Visualiser with the file name
    viewer->setBackgroundColor (0, 0, 0);                   // RGB 000->Black
    viewer->addPointCloud<pcl::PointXYZ> (newCloud, file);  // Cloud pointer and the name of the cloud
    viewer->addPointCloud<pcl::PointXYZ> (removedPoints, removedCloudName);  // Cloud pointer and the name of the cloud
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, file);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, removedCloudName);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, removedCloudName); //draw the removed points in red
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0, 0, 8000,    0, 0, 0,   0, 0, 0,  0);
    // viewer->addCoordinateSystem(500.0);

    // Wait so the user can contemplate the amazing cloud
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
    }
}

//Function to demonstrate PassThrough filtering
int zFilter(int argc, char** argv)
{
    // Check input
    if (argc != 4) {
        printf("Usage: %s FILE.pcd ZMIN ZMAX\n",argv[0]);
        exit(1);
    }
    const char* file = argv[1];
    const double zMin = std::strtod(argv[2],NULL);
    const double zMax = std::strtod(argv[3],NULL);

    // Reading file
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *cloudPtr) == -1) // Load the file into the cloud pointer
    {
        PCL_ERROR ("Couldn't read file: %s \n",file);
        return (-1);
    }

    std::cout << "Sensor Origin: " << cloudPtr->sensor_origin_[0] << " " 
    							   << cloudPtr->sensor_origin_[1] << " "
    							   << cloudPtr->sensor_origin_[2] << " "
    							   << cloudPtr->sensor_origin_[3] << " "
    							   << std::endl; 

    std::cout << "Number of points before filtering: " << cloudPtr->points.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud (new pcl::PointCloud<pcl::PointXYZ>);

    //this PassThrough filter filters points based on their Z value
    //in this case, z starts at 0 from the camera
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(cloudPtr);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(zMin,zMax);
    filter.filter(*newCloud);

    std::cout << "Number of points after filtering: " << newCloud->points.size() << std::endl;

    //create a new cloud and use it so store the points that the filter removed
    pcl::PointCloud<pcl::PointXYZ>::Ptr removedPoints (new pcl::PointCloud<pcl::PointXYZ>);
    filter.setNegative(true);
    filter.filter(*removedPoints);

    const char* removedCloudName = "removedPoints";

    // Visualising cloud
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (file));   // Visualiser with the file name
    viewer->setBackgroundColor (0, 0, 0);                   // RGB 000->Black
    viewer->addPointCloud<pcl::PointXYZ> (newCloud, file);  // Cloud pointer and the name of the cloud
    // viewer->addPointCloud<pcl::PointXYZ> (removedPoints, removedCloudName);  // Cloud pointer and the name of the cloud
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, file);
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, removedCloudName);
    // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, removedCloudName); //draw the removed points in red
    viewer->initCameraParameters ();
    viewer->setCameraPosition(0, 0, 8000,    0, 0, 0,   0, 0, 0,  0);
    // viewer->addCoordinateSystem(500.0);

    // Wait so the user can contemplate the amazing cloud
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
    }
}