#ifndef _READ_PCD_H_
#define _READ_PCD_H_

#include <bits/stdc++.h>
// #include <iostream>
// #include <thread>
// #include <std/vector.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
// #include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <filesystem>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>
#include "csv.h"

using namespace std::chrono_literals;

class NormalityDetection
{
public:
    bool dirFiltered = false;
    std::vector<std::string>
        allPcdList,
        rawList,
        filteredList;
    
public:
    NormalityDetection();
    
    ~NormalityDetection();
    
    void readDir(std::string dirIn, bool verbose);
    
    void getFileList(std::string searchString, std::vector<std::string> &fileListOut, std::string dirIn, bool verbose);
    
    void filterPcd(std::vector<std::string> pcdFileList, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    void readPcd(std::string pcdFile, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool verbose);
    
    pcl::visualization::PCLVisualizer::Ptr createVisualizer(std::string viewerTitle);
          
    void passThroughFilterZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, bool verbose);
    
    void passThroughFilterX(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, bool verbose);
    
    void passThroughFilterY(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, bool verbose);
    
    void writePcd(std::string dir, std::string pcdOut, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool verbose);
    
    void segPlane(pcl::PointIndicesPtr inliersIndices, pcl::ModelCoefficientsPtr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, bool verbose);
    
    void pointCloudFromIndices(pcl::PointCloud<pcl::PointXYZ>::Ptr mainCloud, pcl::PointIndices extractIndices, pcl::PointCloud<pcl::PointXYZ>::Ptr extractCloud, bool verbose);
    
    int getLargestClusterIndex(std::vector<pcl::PointIndices> clusters, bool verbose);
        
    void estimateNormalsIntegralImages(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::Normal>::Ptr cloudNormals, bool verbose);
    
    void estimateNormalsOMP(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::Normal>::Ptr cloudNormals, bool verbose);
    
    void regionGrowingClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::Normal>::Ptr cloudNormals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colouredCloud, std::vector<pcl::PointIndices> &clusters, bool verbose);
    
    pcl::PointCloud<pcl::PointXYZ> XYZRGBtoXYZ(pcl::PointCloud<pcl::PointXYZRGB> cloudIn);

    pcl::PointCloud<pcl::PointXYZ> extractLargestCluster(std::pair<std::vector<pcl::PointIndices>, pcl::PointCloud<pcl::PointXYZRGB>> regionGrowing);

    Eigen::Matrix3f calcRotationMatrix(Eigen::Vector3f planeNormal, bool verbose);    
    
    short getIndex(std::vector<std::string> v, std::string element);
    
    void printProgress(std::vector<std::string> v, std::string element);
};


#endif // _READ_PCD_H_
