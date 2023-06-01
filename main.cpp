#include "normality_detection/NormalityDetection.h"

// Either use all files in a directory
// or use only a single .pcd file (if commented)
// #define DIR

#ifndef DIR
// use pcl visualizer in case of dealing with a single .pcd file
#define VIS
#endif // DIR

int main(int argc, char **argv)
{
    // Init
    std::string dirIn, output_csv;

// if DIR is not defined --> pass a .pcd file to the executable
#ifndef DIR
    if (!argv[1])
    {
        std::cerr << "No .pcd file passed in arguments" << std::endl;
        exit(1);
    }
#endif // DIR

// if DIR is defined --> pass a directory of .pcd files to the executable
#ifdef DIR
    if (!argv[1])
    {
        std::cerr << "No directory passed in arguments" << std::endl;
        exit(1);
    }
#endif // DIR

// check for output .csv file name only if we are passing a directory
#ifdef DIR
    if (!argv[2])
    {
        std::cerr << "Output file [.csv] is not passed in the arguments list" << std::endl;
        exit(1);
    }
    output_csv = argv[2];
#endif // DIR

    dirIn = argv[1];

    NormalityDetection nd;

    // Reference Plane (for visualization only)
    pcl::ModelCoefficientsPtr refPlaneCoeffs(new pcl::ModelCoefficients);
    // 0x + 0y + 1z = 0 -> z normal on xy plane
    Eigen::Vector3f refNormal = Eigen::Vector3f::UnitZ();
    refPlaneCoeffs->values.push_back(refNormal[0]);
    refPlaneCoeffs->values.push_back(refNormal[1]);
    refPlaneCoeffs->values.push_back(refNormal[2]);
    refPlaneCoeffs->values.push_back(0);

#ifdef DIR
    // create .csv file
    int i = 1;
    csvfile csv("../" + output_csv + ".csv"); // throws exceptions!
    // Header
    csv << "PCD_file"
        << "x"
        << "y"
        << "z" << endrow;

    // read all files in the given directory
    nd.readDir(dirIn, true);
    // Check if the directory was already filtered.
    // If yes, skip the preprocessing/filtration routine
    // and read the filtered .pcd files directly
    std::vector<std::string> list;

    if (nd.dirFiltered)
        list = nd.filteredList;
    else
        list = nd.rawList;

    for (const auto &pcdFile : list)
    {
#endif // DIR

        // IO
        pcl::PointCloud<pcl::PointXYZ>::Ptr rawCloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Preprocessing
        pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessedCloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Filtering
        pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colouredCloud(new pcl::PointCloud<pcl::PointXYZRGB>); // for visualization only
        std::vector<pcl::PointIndices> regionGrowingClusters;                                        // list of all cluster indices
        int idx;
        pcl::PointCloud<pcl::PointXYZ>::Ptr largestClusterCloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Plane fitting
        pcl::PointIndicesPtr planeInliers(new pcl::PointIndices);
        pcl::ModelCoefficientsPtr planeCoeffs(new pcl::ModelCoefficients);

#ifndef DIR
        std::string pcdFile = dirIn + "";
#endif // DIR

#ifdef DIR
        if (!(nd.dirFiltered))
        {
#endif
            // 1. [IO]/ Reading .pcd file(s)
            nd.readPcd(pcdFile + ".pcd", rawCloud, true);

            // 2. [Preprocessing]/ specifying a region of interest
            
            nd.passThroughFilterZ(rawCloud, preprocessedCloud, true);
            nd.passThroughFilterX(preprocessedCloud, true);
            nd.passThroughFilterY(preprocessedCloud, true);
            nd.downsampleCloud(preprocessedCloud, true);

            // 3. [Filtering]/ Currently using region growing clustering. We might need to implement/try other algos
            // TODO: implement a standalone function for the region growing algo containing step [3]

            // Apply Region Growing clustering algo
            // 3.1 Get cloud normals
            nd.estimateNormalsOMP(preprocessedCloud, cloudNormals, true);

            // 3.2 Get all clusters out of region growing algo
            nd.regionGrowingClustering(preprocessedCloud, cloudNormals, colouredCloud, regionGrowingClusters, true);

            // 3.3 Get largest cluster
            // hypothetically, the cluster that's most likely representing the plane
            // after removing most of the outliers and noise
            idx = nd.getLargestClusterIndex(regionGrowingClusters, true);
            nd.pointCloudFromIndices(preprocessedCloud, regionGrowingClusters[idx], largestClusterCloud, true);

#ifdef DIR
            // IO]/ Write largest cluster to a new .pcd file to avoid filtering everytime
            nd.writePcd("", pcdFile.substr(0, pcdFile.find("raw")) + "filtered.pcd", largestClusterCloud, true);
        }
        else
        {
            // read .pcd files that end with 'filtered' and store in largestClusterCloud
            nd.readPcd(pcdFile + ".pcd", largestClusterCloud, true);
        }
#endif // DIR

        // 4. [Plane fitting]/ get a plane using RANSAC
        // fit a plane using the largest cluster we got in the previous step
        nd.segPlane(planeInliers, planeCoeffs, largestClusterCloud, true);
        // 4.1 get the normal to the fitted plane
        Eigen::Vector3f planeNormal;
        planeNormal[0] = planeCoeffs->values[0], planeNormal[1] = planeCoeffs->values[1], planeNormal[2] = planeCoeffs->values[2];

        // 5. [Pose estimation]/ get a rotation matrix(->then euler angles) based on the
        Eigen::Matrix3f rot_mat = nd.calcRotationMatrix(planeNormal, true);
        Eigen::Vector3f euler = rot_mat.eulerAngles(0, 1, 2) * 180.0 / M_PI;

        std::cout << std::endl
                  << std::setprecision(6) << euler[0] << ", " << euler[1] << ", " << euler[2] << std::endl
                  << std::endl;

        // // calculate the resultant angle (distance) between
        // float angle = std::acos(refNormal.dot(planeNormal) / (refNormal.norm() * planeNormal.norm())) * 180 / M_PI;

        // // std::cout << pcdFile.substr(0, pcdFile.find("raw")) + "filtered.pcd" << ": " << angle << std::endl;

        // std::cout << "angle = " << angle << std::endl;
#ifdef DIR
        // 6. [IO]/[Postprocessing]
        csv << pcdFile.substr(0, pcdFile.find("raw")) + "filtered.pcd" << euler[0] << euler[1] << euler[2] << endrow;

        nd.printProgress(list, pcdFile);

        // write a new file as a flag stating that the directory is filtered. you need to delete this file if you want to re-filter in dir mode
        std::ofstream outfile(dirIn + "filtered_directory");
        outfile << "" << std::endl;
        outfile.close();
    }
#endif // DIR

#ifdef VIS
    pcl::visualization::PCLVisualizer::Ptr visRaw = nd.createVisualizer("Raw");
    visRaw->addPointCloud(rawCloud, "raw_cloud");
    visRaw->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "raw_cloud");

    pcl::visualization::PCLVisualizer::Ptr visNormals = nd.createVisualizer("Normals");
    visNormals->addPointCloud<pcl::PointXYZ>(preprocessedCloud, "preprocessed_cloud");
    visNormals->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "preprocessed_cloud");
    visNormals->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(preprocessedCloud, cloudNormals, 50, 0.0085, "normals_cloud");
    visNormals->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1.0, 0.0, 1.0, "normals_cloud");
    visNormals->setPointCloudRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 2, "normals_cloud");

    pcl::visualization::PCLVisualizer::Ptr visRegionGrowing = nd.createVisualizer("Region Growing");
    visRegionGrowing->addPointCloud<pcl::PointXYZ>(largestClusterCloud, "largest_cluster");
    visRegionGrowing->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "largest_cluster");
    visRegionGrowing->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "largest_cluster");
    pcl::PointXYZ p1_fitted(planeNormal.x(), planeNormal.y(), planeNormal.z());
    visRegionGrowing->addLine(pcl::PointXYZ(0, 0, 0), p1_fitted, 1.0, 1.0, 0.0, "fitted_plane_normal");
    visRegionGrowing->addPlane(*planeCoeffs, "fitted_plane");
    visRegionGrowing->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 5.0, "fitted_plane_normal");
    visRegionGrowing->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "fitted_plane");
    pcl::PointXYZ p1_ref(refNormal.x(), refNormal.y(), refNormal.z());
    visRegionGrowing->addLine(pcl::PointXYZ(0, 0, 0), p1_ref, 0.0, 1.0, 0.0, "ref_plane_normal");
    visRegionGrowing->addPlane(*refPlaneCoeffs, "ref_plane");
    visRegionGrowing->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_LINE_WIDTH, 5.0, "ref_plane_normal");
    visRegionGrowing->setShapeRenderingProperties(pcl::visualization::RenderingProperties::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "ref_plane");

    pcl::visualization::PCLVisualizer::Ptr visRegionGrowingColoured = nd.createVisualizer("Region Growing Coloured Segments");
    visRegionGrowingColoured->addPointCloud<pcl::PointXYZRGB>(colouredCloud, "coloured_cloud");
    visRegionGrowingColoured->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "coloured_cloud");

    while (!visRaw->wasStopped()) // || !nd.viewerStatus(viewer2))
    {
        visRaw->spinOnce(100);
        visNormals->spinOnce(100);
        visRegionGrowing->spinOnce(100);
        visRegionGrowingColoured->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }
#endif // VIS

    return 0;
}